import os
import sys
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

def plot_trajectory_xy(csv_path):
        print(csv_path)
        try:
            traj = pd.read_csv(csv_path, comment="#") 
            traj.columns = ['x', 'y']
            fig = plt.figure(figsize=(8,8))
            plt.plot(traj['x'], traj['y'], label="TRAJECTORY")
            plt.legend(loc="upper right")
            plt.grid()
            plt.xlabel("x")
            plt.ylabel("y")
            plt.axis('equal')
            plt.show()
            out_path = csv_path.replace(".csv", ".png")
            print(out_path)
            fig.savefig(out_path, bbox_inches='tight', dpi=250)
            plt.close(fig)
        except: 
            print("Error in reading " + str(csv_path))


if __name__ == '__main__':
    if len(sys.argv) == 2:
        csv_path = sys.argv[1]
        # print(csv_path)

        if not csv_path.endswith(".csv"):
            print("csv extension needed")
            sys.exit(1)


    else:
        path = Path(__file__).parent
        print(path)
        dirs = os.listdir(path)
        print(dirs)
        for file in dirs:
            if file.endswith('.csv'):
                file1 = os.path.join(path, file)
                print(file1)
                plot_trajectory_xy(file1)


        # print("Usage:\n{} csv_path".format(sys.argv[0]))
        # sys.exit(1)