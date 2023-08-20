# VINS-Mono

Repo for the `VINS-Mono` code inside docker `icra2018/vins-mono:latest`. For more details, see `README.ipynb`.

## Usage

### Launch

```bash
# use jupyter
nvidia-docker run -u jovyan -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/shuyuan-19/dataset/Euroc:/dataset icra2018/vins-mono:v1 jupyter lab --no-browser --ip=0.0.0.0 --NotebookApp.token='' --allow-root
# or launch the terminal (better)
nvidia-docker run -u jovyan -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/shuyuan-19/dataset/Euroc:/dataset icra2018/vins-mono:v1 
```

## Reference

:star:[evo评测VINS-MONO---代码修改、数据格式转换、数据测试](https://blog.csdn.net/xiaojinger_123/article/details/120141017)

:star:[icra/vins-mono docker](https://hub.docker.com/r/icra2018/vins-mono)

[evo评测VINS-MONO---指标解析、算法精度分析（数据集）](https://blog.csdn.net/xiaojinger_123/article/details/120269185)

[VINS-Mono运行与评测](https://rupingcen.blog.csdn.net/article/details/110485772)

[EVO评估Vins-Mono](https://blog.csdn.net/weixin_41954990/article/details/127845403)
