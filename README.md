This is the webpage of the open-source packages of our paper submitted to
ICRA2016. Our implementation of dense tracking is built on the top of the
open-source and vectorized implementation of [1]. Our package is compatible with
the standard driver of VI-Sensor and ROS version of indigo. OpenCV is needed.

The high resolution video of our submitted paper is: http://1drv.ms/1QlCTbl

If you can not visit Microsoft Onedrive, please add
'''
134.170.108.26 onedrive.live.com
134.170.109.48 skyapi.onedrive.live.com
'''
in the end of windows/system32/drivers/etc/hosts. (windows user)

We have uploaded the experimental data shown in the video. All the raw data,
estimatior output, UKF smoothing output, control command and etc. are included
in the rosbags. To download it, please visit:

* Cross-pattern: http://1drv.ms/1NQ0rpY
* Figure-eight: http://1drv.ms/1NQ0uSG

If you use the code, please cite

'''latex
@INPROCEEDINGS{
    author={Y. Ling and S. Shen},
    booktitle={2015 IEEE International Conference on Robotics and Biomimetics (ROBIO)},
    title={Dense visual-inertial odometry for tracking of aggressive motions},
    year={2015},
    pages={576-583},
    doi={10.1109/ROBIO.2015.7418830},
    month={Dec},
}
'''

our paper "Aggresive Quadrotor Flight Using
Dense Visual-Inertial Fusion", submitted to Proc. of the {IEEE} Intl. Conf. on
Robot. and Autom., 2016.

For more questions, please contact ylingaa at connect dot ust dot hk .

[1] LSD-SLAM: Large-Scale Direct Monocular SLAM (J. Engel, T. Schöps, D.
Cremers), In European Conference on Computer Vision (ECCV), 2014.
