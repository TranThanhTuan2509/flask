"""Build seed-controlled upstream drivers without editing vendored sources.

Only RNG initialization changes. Generated sources, a unified diff, commands,
and input/output hashes are retained in build/seeded_baselines. Search, dynamics,
costs, stopping rules, and trajectory optimization remain upstream code.
"""
import difflib
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]
BUILD=ROOT/'build/baselines'
OUTPUT=ROOT/'build/seeded_baselines'
UPSTREAM=ROOT/'third_party/dynoplan'

HEADER=r'''#pragma once
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <ompl/util/RandomNumbers.h>
namespace flask_benchmark {
inline unsigned int seed() {
    const char* text=std::getenv("FLASK_BENCHMARK_SEED");
    if(!text || !*text) throw std::invalid_argument("FLASK_BENCHMARK_SEED is required");
    std::string value(text);
    if(value.find_first_not_of("0123456789")!=std::string::npos)
        throw std::invalid_argument("Seed must be a positive integer");
    unsigned long long number=std::stoull(value);
    if(number==0 || number>std::numeric_limits<unsigned int>::max())
        throw std::invalid_argument("Seed must be in [1, UINT_MAX]");
    return static_cast<unsigned int>(number);
}
inline void seed_c_and_ompl() {
    const auto value=seed();std::srand(value);ompl::RNG::setSeed(value);
}
}
'''


def replace_once(text,old,new):
    if text.count(old)!=1:
        raise RuntimeError('Upstream source drift: expected RNG initialization exactly once')
    return text.replace(old,new,1)


def compile_flags(target):
    values={}
    for line in (BUILD/f'CMakeFiles/{target}.dir/flags.make').read_text().splitlines():
        if ' = ' in line:
            key,value=line.split(' = ',1);values[key]=shlex.split(value)
    return sum((values.get(key,[]) for key in ('CXX_DEFINES','CXX_INCLUDES','CXX_FLAGS')),[])


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    OUTPUT.mkdir(parents=True,exist_ok=True)
    (OUTPUT/'flask_seed.hpp').write_text(HEADER)
    commands=[];patches=[];inputs={};outputs={}
    specifications=[
        ('main_sst','src/ompl/main_sst.cpp'),
        ('main_idbastar','src/idbastar/main_idbastar.cpp'),
        ('dbastar','src/dbastar/dbastar.cpp'),
    ]
    for target,relative in specifications:
        original=UPSTREAM/relative;text=original.read_text();inputs[str(original.relative_to(ROOT))]=digest(original)
        if target=='dbastar':
            old='''  std::mt19937 g = std::mt19937{std::random_device()()};
  if (options_dbastar.fix_seed) {
    expander.seed(0);
    g = std::mt19937{0};
    srand(0);
  } else {
    srand(time(0));
  }'''
            new='''  const auto registry_seed = flask_benchmark::seed();
  std::mt19937 g{registry_seed};
  expander.seed(registry_seed);
  std::srand(registry_seed);'''
            modified=replace_once(text,old,new)
        else:
            modified=replace_once(text,'srand(time(0));','flask_benchmark::seed_c_and_ompl();')
        modified='#include "flask_seed.hpp"\n'+modified
        generated=OUTPUT/Path(relative).name;generated.write_text(modified)
        patches.extend(difflib.unified_diff(text.splitlines(True),modified.splitlines(True),
                                          fromfile=str(original.relative_to(ROOT)),tofile=str(generated.relative_to(ROOT))))
        command=['/usr/bin/c++',*compile_flags(target),'-I',str(OUTPUT),'-c',str(generated),'-o',str(OUTPUT/(generated.name+'.o'))]
        commands.append(command);subprocess.run(command,cwd=BUILD,check=True)
    (OUTPUT/'rng_initialization.patch').write_text(''.join(patches))
    archive=OUTPUT/'libdbastar.a';shutil.copyfile(BUILD/'libdbastar.a',archive)
    command=['ar','r',str(archive),str(OUTPUT/'dbastar.cpp.o')]
    commands.append(command);subprocess.run(command,check=True)
    for target,relative in specifications[:2]:
        command=shlex.split((BUILD/f'CMakeFiles/{target}.dir/link.txt').read_text())
        for i,argument in enumerate(command):
            if argument.endswith(f'/{Path(relative).name}.o'):
                command[i]=str(OUTPUT/(Path(relative).name+'.o'))
            if argument=='libdbastar.a':command[i]=str(archive)
            if i and command[i-1]=='-o':command[i]=str(OUTPUT/target)
        commands.append(command);subprocess.run(command,cwd=BUILD,check=True)
        outputs[str((OUTPUT/target).relative_to(ROOT))]=digest(OUTPUT/target)
    outputs.update(build_primitives(commands,patches,inputs))
    (OUTPUT/'rng_initialization.patch').write_text(''.join(patches))
    record=dict(purpose='upstream_algorithms_with_registry_controlled_rng_initialization',
                rng_environment_variable='FLASK_BENCHMARK_SEED',seed_range=[1,2**32-1],
                input_sha256=inputs,output_sha256=outputs,commands=commands,
                upstream_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=UPSTREAM,text=True).strip())
    (OUTPUT/'build_manifest.json').write_text(json.dumps(record,indent=2)+'\n')


def build_primitives(commands,patches,inputs):
    """v10: the motion-primitive generator (iDb-A* banks) seeds from time(0)/random_device upstream, so banks were
    not reproducible. Only those initializations are replaced by FLASK_BENCHMARK_SEED; generation is unchanged."""
    outputs={}
    specs=[('main_primitives','src/motion_primitives/main_primitives.cpp',
            [('srand((unsigned int)time(0));','std::srand(flask_benchmark::seed());  // no OMPL in this target',1),
             ('std::mt19937 g(rd());','std::mt19937 g(flask_benchmark::seed());',2)]),
           ('motion_primitives','src/motion_primitives/motion_primitives.cpp',
            [('std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()','std::mt19937 gen(flask_benchmark::seed());',1)])]
    objects={}
    for target,relative,edits in specs:
        original=UPSTREAM/relative;text=original.read_text();inputs[str(original.relative_to(ROOT))]=digest(original)
        modified=text
        for old,new,count in edits:
            if modified.count(old)!=count:raise RuntimeError(f'Upstream source drift in {relative}: {old!r}')
            modified=modified.replace(old,new)
        modified='#include "flask_seed.hpp"\n'+modified
        generated=OUTPUT/Path(relative).name;generated.write_text(modified)
        patches.extend(difflib.unified_diff(text.splitlines(True),modified.splitlines(True),
                                          fromfile=str(original.relative_to(ROOT)),tofile=str(generated.relative_to(ROOT))))
        objects[target]=OUTPUT/(generated.name+'.o')
        command=['/usr/bin/c++',*compile_flags(target),'-I',str(OUTPUT),'-c',str(generated),'-o',str(objects[target])]
        commands.append(command);subprocess.run(command,cwd=BUILD,check=True)
    archive=OUTPUT/'libmotion_primitives.a';shutil.copyfile(BUILD/'libmotion_primitives.a',archive)
    command=['ar','r',str(archive),str(objects['motion_primitives'])]
    commands.append(command);subprocess.run(command,check=True)
    command=shlex.split((BUILD/'CMakeFiles/main_primitives.dir/link.txt').read_text())
    for i,argument in enumerate(command):
        if argument.endswith('/main_primitives.cpp.o'):command[i]=str(objects['main_primitives'])
        if argument=='libmotion_primitives.a':command[i]=str(archive)
        if argument=='libdbastar.a':command[i]=str(OUTPUT/'libdbastar.a')
        if i and command[i-1]=='-o':command[i]=str(OUTPUT/'main_primitives')
    commands.append(command);subprocess.run(command,cwd=BUILD,check=True)
    outputs[str((OUTPUT/'main_primitives').relative_to(ROOT))]=digest(OUTPUT/'main_primitives')
    return outputs


if __name__=='__main__':main()
