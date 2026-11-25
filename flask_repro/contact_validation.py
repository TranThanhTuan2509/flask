"""Classify robot/obstacle contacts without conflating IDs and sample masks."""


def obstacle_contacts(contact_pairs,robot_geometries,static_geometries,moving_geometries):
    static_hit=moving_hit=False
    for first,second in contact_pairs:
        if first in robot_geometries:other=second
        elif second in robot_geometries:other=first
        else:continue
        static_hit|=other in static_geometries
        moving_hit|=other in moving_geometries
    return static_hit,moving_hit
