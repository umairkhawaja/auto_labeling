import os
import argparse
import numpy as np 

def parse_args():
    parser = argparse.ArgumentParser('filter')
    parser.add_argument('--scan', type=str, default=None, help='Path to the training scan [default: None]')
    parser.add_argument('--dest_dir', type=str, default=None, help='Path to save the scan [default: None]')
    parser.add_argument('--filter_threshold', type=float, default=0.5, help='Threshold to filter the points [default: 0.5]')

    return parser.parse_args()


def main(args):

    labels_column = 4   #3: gt, 4: predicted 

    data = np.loadtxt(args.scan)
    original_size = len(data)
    # Remove Duplicate columns from 2D NumPy Array
    data = np.unique(data, axis=0)  #0: raw, 1: column
    labels = data[:,labels_column]
    filter_idx = np.where(labels > args.filter_threshold)
    data = np.delete(data, filter_idx, 0)

    labels = data[:,labels_column]
    filter_idx = np.where(labels <= 0.05)
    data = np.delete(data, filter_idx, 0)

    new_size = len(data)

    points, labels = data[:,:3], data[:,labels_column]

    data = np.column_stack((points, labels)) 

    scan_id = os.path.basename(args.scan)
    scan_id = scan_id[:-19] #to remove _points_gt_pred.asc
    print("Saving %s: %d > %d" % (scan_id, original_size, new_size))
    np.savetxt(os.path.join(args.dest_dir, scan_id + ".asc") , data, fmt='%f')

if __name__ == '__main__':
    args = parse_args()
    main(args)
