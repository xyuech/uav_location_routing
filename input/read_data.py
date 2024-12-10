"""
Author: xych
Date:   2024-12-09
Usage:  Import location data
"""
import input.config as config
import csv
from typing import List

def read_locations(filename: str) -> List:
    locations = []
    with open(config.Path.HOME+'/input/'+filename, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            locations.append(
                tuple([float(num) for num in row])
            )
    return locations