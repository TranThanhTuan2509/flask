"""Claim labels: a row below the validity gate is UNSOLVED_IN_BUDGET only if no returned path failed the check."""
from benchmarks.report_v10 import MIN_VALID, verdict


def test_unsolved_in_budget_vs_invalid():
    low = MIN_VALID - .1
    assert verdict(True, True, low, ['WITHIN_BAND'], True, returned_invalid=0) == 'UNSOLVED_IN_BUDGET'
    assert verdict(True, True, low, ['WITHIN_BAND'], True, returned_invalid=1) == 'DIFFERENT_INVALID'
    assert verdict(True, True, low, ['WITHIN_BAND'], True) == 'DIFFERENT_INVALID'          # unknown -> conservative
    assert verdict(True, True, 1., ['WITHIN_BAND'], True, returned_invalid=0) == 'REPRODUCED'
    assert verdict(True, False, low, ['WITHIN_BAND'], True, returned_invalid=0) == 'UNCONTROLLED'
