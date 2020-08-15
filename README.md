# Kalligraph
## About
Kalligraph is supposed to become a utility library for rendering text and vector graphics on the GPU.
Features are supposed to be optional and can be enabled as needed.

The first feature I am working on is mesh generation and shaders for the technique presented by Loop and Blinn in 2005 [in GPU Gems 3](https://developer.nvidia.com/gpugems/gpugems3/part-iv-image-effects/chapter-25-rendering-vector-art-gpu) ([original paper](https://www.microsoft.com/en-us/research/wp-content/uploads/2005/01/p1000-loop.pdf), [presentation](https://www.youtube.com/watch?v=2OAPHn_YWGA)).

Currently all there is, is some brute force triangulation that still tends to leave some holes and likely also has other issues. (I've looked at different libraries to use, but they are either only free for non commercial projects or other restrictions. I am considering to eventually try poly2tri if my solution ends up too slow).
