# SMART-V
SMART-V is a modified [RI5CY](https://github.com/openhwgroup/cv32e40p), now known as CV32E40P processor.
This repo contains the old RI5CY code, so it might not be fully compatible with CV32E40P.

SMART-V has security extensions for memory protection tailored towards a microkernel-based OS (SmartOS), which is not yet publicly available.
Also, my current paper describing SMART-V is not yet published, so I do not disclose many details.

## Test Files
The manual `SMART-V Documentation` is meant to describe the design decisions behing the HW/SW co-design in details.
For now, it is only a collection of simple test benches (in `test_files`) executed on SMART-V with no security modules activated.
It is a big work-in-progress.

To run the test files, see `test_files/README.md`

