import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl

def get_args_parser():
    parser = argparse.ArgumentParser('args', add_help=False)
    parser.add_argument('--csv', default='../sample_output/kalman_recursive.csv', type=str)
    parser.add_argument('--csv2', default='../sample_output/kalman_gain.csv', type=str)
    parser.add_argument('--csv3', default='../sample_output/kalman_joseph.csv', type=str)
    return parser


def plot(__csv, __obs_ids, __true_value, __meas_value, __corr_value, __diff):
    mpl.rcParams['text.usetex'] = True
    mpl.rcParams['legend.fontsize'] = 14

    f,ax = plt.subplots(2,2,figsize=(30,15))
    f.suptitle(__csv, fontsize=16)
    for __obs_id in __obs_ids:
        ax[0,0].scatter(__true_value[__obs_id]['x'],__true_value[__obs_id]['y'],s=3,c='r',marker='o',alpha=1.)
        ax[0,0].scatter(__meas_value[__obs_id]['x'],__meas_value[__obs_id]['y'],s=3,c='b',marker='P',alpha=0.3)
        ax[0,0].scatter(__corr_value[__obs_id]['x'],__corr_value[__obs_id]['y'],s=5,c='g',marker='^',alpha=0.3)
        ax[0,0].legend(('True value','measured value','corrected value'),loc='upper right')
        #'My Title\n' + r'$\alpha - \omega$ are LaTeX Markup'
        ax[0,0].set_title("true and measured value of the obstacles' positions detected by lidar\n and their corrected values by kalman filter",fontsize=14)
        ax[0,0].set_xlabel(r'global frame position $x - [m]$')
        ax[0,0].set_ylabel(r'global frame position $y - [m]$')
        
        ax[0,1].scatter(__true_value[__obs_id]['vx'],__true_value[__obs_id]['vy'],s=3,c='r',marker='o',alpha=1.)
        ax[0,1].scatter(__meas_value[__obs_id]['vx'],__meas_value[__obs_id]['vy'],s=3,c='b',marker='p',alpha=0.3)
        ax[0,1].scatter(__corr_value[__obs_id]['vx'],__corr_value[__obs_id]['vy'],s=5,c='g',marker='^',alpha=0.3)
        ax[0,1].legend(('true value','measured value','corrected value'),loc='upper right')
        ax[0,1].set_title("True and measured value of the obstacles' velocities detected by lidar \n and their corrected values by kalman filter",fontsize=14)
        ax[0,1].set_xlabel(r'global frame velocity $x - [m /s]$')
        ax[0,1].set_ylabel(r'global frame velocity $y - [m /s]$')

        __idx = [x for x in range(len(__diff[__obs_id]['x']))]
        if __obs_id != 1:
            continue
        ax[1,0].plot(__idx, __diff[__obs_id]['x'],color='r')
        ax[1,0].plot(__idx, __diff[__obs_id]['y'],color='b')
        ax[1,0].legend((r'difference in position in $x$ axis',r'difference in position in $y$ axis'),loc='upper right')
        #'My Title\n' + r'$\alpha - \omega$ are LaTeX Markup'
        ax[1,0].set_title("Difference of the position in true and corrected value in percentage",fontsize=14)
        ax[1,0].set_xlabel(r'each discrete time step $dt - [unit]$')
        ax[1,0].set_ylabel(r'global frame position difference $diff - [\%]$')
        ax[1,0].set_ylim(-5,5)

        ax[1,1].plot(__idx, __diff[__obs_id]['vx'],color='g')
        ax[1,1].plot(__idx, __diff[__obs_id]['vy'],color='y')
        ax[1,1].legend((r'difference in velocity in $x$ axis',r'difference in velocity in y axis'),loc='upper right')
        ax[1,1].set_title("Difference of the velocity in true and corrected value in percentage", fontsize=14)
        ax[1,1].set_xlabel(r'each discrete time step $dt - [unit]$')
        ax[1,1].set_ylabel(r'global frame velocity difference $diff - [\%]$')
    
    plt.savefig(__csv+'.png', bbox_inches='tight')
    plt.show()

def main(args):
    csvs = [args.csv, args.csv2, args.csv3]
    for csv in csvs:
        df = pd.read_csv(csv)
        n = len(df)
        obs_ids = set()
        true_value = {}      
        meas_value = {}
        corr_value = {}
        diff = {}

        for __idx, __row in df.iterrows():
            if __idx == n-1:
                break
            obs_id = __row['obs_id']
            obs_ids.add(obs_id)
            a = __row['true_value'].split()
            b = [*map(float, a)]
            a1 = __row['meas_value'].split()
            b1 = [*map(float, a1)]
            a2 = __row['corr_value'].split()
            b2 = [*map(float, a2)]
            a3 = __row['diff(%)'].split()
            b3 = [*map(float, a3)]

            if obs_id not in true_value:
                true_value[obs_id] = {'x':[b[0]],'y':[b[1]],'vx':[b[2]],'vy':[b[3]]}
                meas_value[obs_id] = {'x':[b1[0]],'y':[b1[1]],'vx':[b1[2]],'vy':[b1[3]]}
                corr_value[obs_id] = {'x':[b2[0]],'y':[b2[1]],'vx':[b2[2]],'vy':[b2[3]]}
                diff[obs_id] = {'x':[b3[0]],'y':[b3[1]],'vx':[b3[2]],'vy':[b3[3]]}

            else:
                true_value[obs_id]['x'].append(b[0])
                true_value[obs_id]['y'].append(b[1])
                true_value[obs_id]['vx'].append(b[2])
                true_value[obs_id]['vy'].append(b[3])

                meas_value[obs_id]['x'].append(b1[0])
                meas_value[obs_id]['y'].append(b1[1])
                meas_value[obs_id]['vx'].append(b1[2])
                meas_value[obs_id]['vy'].append(b1[3])
                
                corr_value[obs_id]['x'].append(b2[0])
                corr_value[obs_id]['y'].append(b2[1])
                corr_value[obs_id]['vx'].append(b2[2])
                corr_value[obs_id]['vy'].append(b2[3])

                diff[obs_id]['x'].append(b3[0])
                diff[obs_id]['y'].append(b3[1])
                diff[obs_id]['vx'].append(b3[2])
                diff[obs_id]['vy'].append(b3[3])
        plot(csv, obs_ids, true_value, meas_value, corr_value, diff)
    return 0

if __name__ == '__main__':
    parser = argparse.ArgumentParser('KF filter output visual analysis', parents=[get_args_parser()])
    args = parser.parse_args()
    #if args.output_dir:
    #    Path(args.output_dir).mkdir(parents=True, exist_ok=True)
    main(args)

