#!/bin/bash

# use this instruction to run smac example branin
./smac --use-instances false --numberOfRunsLimit 100 --pcs-file example_scenarios/branin/params.pcs --algo "python example_scenarios/branin/branin.py" --run-objective QUALITY --log-all-calls true

