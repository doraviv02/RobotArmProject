#!/bin/bash

# Print current working directory
pwd

# Move test.jpg to U_2_Net/test_data/test_images
cp ./test.jpg ./U_2_Net/test_data/test_images/

# Run u2net_test.py
( cd U_2_Net/ && python u2net_test.py )

# Move test.png from U_2_Net/test_data/u2net_results to Segmentation
mv U_2_Net/test_data/u2net_results/test.png ./
