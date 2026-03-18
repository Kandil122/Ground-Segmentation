Build and Run
The project has been successfully built with all new components:

```bash
cd pointcloud_clustering_detector_cuda/build
cmake ..
make -j$(nproc)
```

Run your detector as usual:

```bash
cd pointcloud_clustering_detector_cuda                                                                       
./build/pointcloud_clustering_detector_cuda config/detector.yaml
```