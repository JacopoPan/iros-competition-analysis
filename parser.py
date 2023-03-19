"""Parser.

Notes:
    Main executable script.

Run as:

    $ python3 parsers.py --test_arg <test_arg>

"""
import argparse
import numpy as np
import bagpy
from bagpy import bagreader
import pickle
import csv

def run(
        test_arg,
    ):
    bag = 'exp-data/arg1_2023-03-11-16-25-19.bag'
    b = bagreader(bag)
    print(b.topic_table) # get the list of topics
    csvfiles = []
    for t in b.topics:
        data = b.message_by_topic(t)
        csvfiles.append(data)
    print(csvfiles[0])

    values = 'sim-data/arg/save-flight-get_start-episode-1-03.16.2023_16.45.14/p0.csv'
    with open(values) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for i, row in enumerate(csv_reader):
            print (i, row)

    pkl = 'exp-data/cmd_test_arg1.pkl'
    with open(pkl, 'rb') as f:
        data = pickle.load(f)
    print(data)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Parsing script')
    parser.add_argument('--test_arg',
                        default='',
                        type=str,
                        help='Test argument', metavar='')
    ARGS = parser.parse_args()
    run(**vars(ARGS))