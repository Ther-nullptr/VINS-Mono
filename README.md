# VINS-Mono

Repo for the `VINS-Mono` code inside docker `icra2018/vins-mono:latest`. For more details, see `README.ipynb`.

## Usage

### Launch

launch the docker:

```bash
# use jupyter
$ nvidia-docker run -u jovyan -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/shuyuan-19/dataset/Euroc:/dataset icra2018/vins-mono:v1 jupyter lab --no-browser --ip=0.0.0.0 --NotebookApp.token='' --allow-root
# or launch the terminal (better)
$ nvidia-docker run -u root -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/shuyuan-19/dataset/Euroc:/dataset -v /home/shuyuan-19/dataset/Euroc/output:/output icra2018/vins-mono:v1
```

run the project:

```bash
$ roscore
$ roslaunch vins_estimator euroc.launch
$ roslaunch vins_estimator vins_rviz.launch # optional
$ rosbag play <bag_file>
```

### Change

#### Time Estimate

若需要打开计时模式，打开`vins-mono/feature_tracker/src/feature_tracker.h`，定义`#define SHOW_TIME`

#### Feature Tracker

查找`cv::goodFeaturesToTrack`函数：

```bash
$ grep -r -n "cv::goodFeaturesToTrack" .
./pose_graph/src/keyframe.cpp:96:               cv::goodFeaturesToTrack(image, tmp_pts, 500, 0.01, 10);
./feature_tracker/src/feature_tracker.cpp:160:            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask); // TODO
```

我们要用自定义的函数替换opencv中使用的函数。这一点通过控制`vins-mono/utils/include/MyGoodFeaturesToTrack.h`中的宏`#define CUSTOM_OVERRIDE`来实现。`vins-mono/utils/include/MyGoodFeaturesToTrack.cpp`为自定义函数主体。

#### Equation Solver

求解方程的步骤位于`vins-mono/pose_graph/src/pose_graph.cpp`中。`ceres::Solve(options, &problem, &summary);`为正式求解方程的步骤。

#### Matrix Extractor

在`vins-mono/pose_graph/src/pose_graph.cpp`中，通过`evaluateBA(problem, summary)`来获取Report，保存residual/gradient/jacobian/hessian.



## Evaluation

```bash
$ evo_ape euroc /dataset/<task_name>/mav0/state_groundtruth_estimate0/data.csv vins_result_loop.txt -va --plot --plot_mode xyz --save_results <task_name>.zip # get the data and visualization
$ evo_res <task_name_a>.zip <task_name_b>.zip -p --save_table table.csv # comparasion
```

## Reference

:star:[evo评测VINS-MONO---代码修改、数据格式转换、数据测试](https://blog.csdn.net/xiaojinger_123/article/details/120141017)

:star:[icra/vins-mono docker](https://hub.docker.com/r/icra2018/vins-mono)

[evo评测VINS-MONO---指标解析、算法精度分析（数据集）](https://blog.csdn.net/xiaojinger_123/article/details/120269185)

[VINS-Mono运行与评测](https://rupingcen.blog.csdn.net/article/details/110485772)

[EVO评估Vins-Mono](https://blog.csdn.net/weixin_41954990/article/details/127845403)
