#!/bin/bash

python3 combine.py 
xxd -i combined_index.html > ../drivers/wifi/combined_html.h
