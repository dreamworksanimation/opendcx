![OpenDCX](http://www.opendcx.org/images/opendcx_logo.png)

[Website](http://www.opendcx.org)


OpenDCX is an open-source C++ library comprising extensions for the OpenEXR library and its deep file format. It adds support for per deep-sample metadata comprising subpixel-masks and surface-type flags, and provides utility functions to manipulate deep pixels with this metadata.
It is developed and maintained by [DreamWorks Animation](http://www.dreamworksanimation.com) for use in rendering and compositing applications typically encountered in feature film production. The technical details of DCX are described in the paper [Improved Deep Image Compositing Using Subpixel Masks](http://research.dreamworks.com)


##Features and Tools

###Efficient Storage Requirements
OpenDCX can dramatically improve image quality with only a modest increase in file size. Deep pixel samples are augmented with a sub-pixel mask and flags that are packed into 3 additional channels that tend to compress very efficiently. Sample code is provided to implement render-side sample consolidation for further reduction in file size.


###Backwards Compatibility
OpenDCX deep samples can coexist with traditional OpenEXR volumetric deep samples. Although these samples lack the sub-pixel mask and hard surface flags, they will be properly composited / flattened by the OpenDCX algorithms.


###Image Manipulation
The OpenDCX toolset includes a DeepTransform class which implements affine image-space transforms of the deep pixel array. In particular, scaling and rotation benefit from the sub-pixel accuracy of the OpenDCX format. This also allows deep images of differing resolutions to be seamlessly combined.


###Pixel reconstruction
The sub-pixel detail of OpenDCX is particularly useful in pixel reconstruction. Deep pixel flattening can be performed per sub-pixel, and each sub-pixel's flattened result can be integrated with other sub-pixel results in a pixel filter.
