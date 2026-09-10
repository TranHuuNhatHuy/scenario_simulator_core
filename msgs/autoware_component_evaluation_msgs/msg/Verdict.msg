# Outcome of one evaluated item.
#
# NOT_OBSERVED and NOT_APPLICABLE are deliberately distinct from FAIL. An interface that was never
# observed may mean a dead component, but it may equally mean a profile that does not launch that
# component; conflating the two is what makes conformance reports untrustworthy.

uint8 UNKNOWN=0
uint8 PASS=1
uint8 FAIL=2
uint8 NOT_OBSERVED=3
uint8 NOT_APPLICABLE=4

uint8 value
