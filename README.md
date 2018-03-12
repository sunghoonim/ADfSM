# ADfSM
(ADfSM: All-around Depth from Small Motion)

Source code and datasets for the paper:

_S. Im, H. Ha, F. Rameau, H.-G. Jeon, G. Choe and I.S. Kweon_, [**All-around Depth from Small Motion with A Spherical Panoramic Camera**](https://drive.google.com/file/d/0BzgPRA7JXoFiMjh1VE8wcDU4ZjA/view) - [**ECCV 2016**](http://www.eccv2016.org/)

- [**Project page**](https://sites.google.com/site/shimrcv/imeccv16)

## Dependency
- [**OpenCV**](http://opencv.org)
- [**Ceres solver**](http://ceres-solver.org)

## How to run
* run main.m


**Important Information**

Frame selection

The current implementation uses only the first 30 frames of your video clip. If you want to try with a different number of images or different sampling rate, please change 'setting.sam_rate'.

Dense matching step

We implemented the function "DenseMatching" to receive a scale for image downsampling and the number of labels for your convenience in testing. (Default: 0.5 scale and 64 labels for quick tests, you can change them ('setting.scaling, 'setting.num_label')

For the depth refinement, we utilized a tree-based depth upsampling approach [1,2].

## Authors

* [Sunghoon Im](https://sites.google.com/site/shimrcv/)
* [Hyowon Ha](https://sites.google.com/site/hyowoncv/)
* [Francois Rameau](https://www.researchgate.net/profile/Francois_Rameau)
* [Hae-Gon Jeon](https://sites.google.com/site/hgjeoncv/)
* [Gyeonmin Choe](http://rcv.kaist.ac.kr/gmchoe/)
* [In So Kweon](http://rcv.kaist.ac.kr/)

&copy; 2017 Sunghoon Im, Korea Advanced Institute of Science and Technology (KAIST)

**IMPORTANT**: If you use this software please cite the following in any resulting publication:

~~~~
@inproceedings{im2016all,
  title={All-Around Depth from Small Motion with a Spherical Panoramic Camera},
  author={Im, Sunghoon and Ha, Hyowon and Rameau, Fran{\c{c}}ois and Jeon, Hae-Gon and Choe, Gyeongmin and Kweon, In So},
  booktitle={European Conference on Computer Vision},
  pages={156--172},
  year={2016},
  organization={Springer}
}
~~~~

## References

1. Yang, Qingxiong. "Stereo matching using tree filtering." IEEE transactions on pattern analysis and machine intelligence 37.4 (2015): 834-846.
2. Yang, Qingxiong. "A non-local cost aggregation method for stereo matching." Computer Vision and Pattern Recognition (CVPR), 2012 IEEE Conference on. IEEE, 2012.

