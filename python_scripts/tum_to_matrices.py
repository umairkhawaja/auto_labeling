import os
import shutil
import argparse
import numpy as np 
from scipy.spatial.transform import Rotation as R

def parse_args():
    parser = argparse.ArgumentParser('transformer')
    parser.add_argument('--trajectory', type=str, default=None, help='Path to the trajectory file [default: None]')
    parser.add_argument('--results_dir', type=str, default=None, help='Directory to save the matrices [default: None]')
    return parser.parse_args()


def main(args):
    trajectory = np.loadtxt(args.trajectory)
    file_name = os.path.basename(args.trajectory)[:-4]
    dist_dir = os.path.join(args.results_dir, file_name)

    try:
        shutil.rmtree(dist_dir)
        os.makedirs(dist_dir)
    except OSError as e:
        os.makedirs(dist_dir)

    for pose in trajectory:
        matrix = np.identity(4)
        matrix_id = str(int(pose[0]*100))
        t, r = pose[1:4].reshape((3,1)), pose[4:]
        r = R.from_quat(r)
        r = r.as_matrix()

        matrix[0:3,0:3] = r
        matrix[0:3,3]=(t)[:,0]

        print("Saving transformation matrix of scan %s to %s ..." % (matrix_id, dist_dir))
        np.savetxt(os.path.join(dist_dir, matrix_id + ".txt") , matrix, delimiter=', ')

if __name__ == '__main__':
    args = parse_args()
    main(args)
