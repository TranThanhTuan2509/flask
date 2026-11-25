"""Negative controls for geometry-ID contact classification."""
from flask_repro.contact_validation import obstacle_contacts


def test_contact_ids_are_classified_in_either_order():
    robot={11,12};static={20};moving={30}
    assert obstacle_contacts([(11,20)],robot,static,moving)==(True,False)
    assert obstacle_contacts([(30,12)],robot,static,moving)==(False,True)
    assert obstacle_contacts([(20,12),(11,30)],robot,static,moving)==(True,True)
    assert obstacle_contacts([(11,12),(20,30),(1,2)],robot,static,moving)==(False,False)
