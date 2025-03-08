#!/bin/bash

python3 combine.py 
xxd -i combined_index.html > ../tasks/combined_html.h
