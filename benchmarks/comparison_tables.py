"""Regenerate English comparison tables from archived numerical measurements."""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
from .research_summary import ROOT,build_report

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--source',type=Path,default=ROOT/'results/research');parser.add_argument('--output',type=Path,default=ROOT/'results/figures');args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'historical_comparison.md').write_text(build_report(args.source))
    rows=[]
    for path in sorted(args.source.glob('mbm_panda*.csv')):
        df=pd.read_csv(path)
        if {'solved','total_ms'}<=set(df):
            successes=df[df.solved.astype(bool)];rows.append([path.stem.replace('mbm_panda_',''),f'{len(successes)}/{len(df)}',f'{successes.total_ms.median():.3f}',f'{successes.total_ms.mean():.3f}'])
    fig,ax=plt.subplots(figsize=(12,2+len(rows)*.45));ax.axis('off');ax.table(cellText=rows,colLabels=['Historical configuration','Solved / attempts','Median total ms','Mean total ms'],loc='center',cellLoc='left',colWidths=[.55,.15,.15,.15])
    ax.set_title('Panda reconstruction: historical measurements\nSuccess-conditioned timings; different conditions from the paper',pad=20)
    for ext in ('png','pdf'):fig.savefig(args.output/f'historical_panda_comparison.{ext}',dpi=240,bbox_inches='tight')
    plt.close(fig)
if __name__=='__main__':main()
