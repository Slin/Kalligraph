# Kalligraph
## About
Kalligraph is a utility library for rendering vector graphics on the GPU.

It does the mesh generation for the technique presented by Loop and Blinn in 2005 [in GPU Gems 3](https://developer.nvidia.com/gpugems/gpugems3/part-iv-image-effects/chapter-25-rendering-vector-art-gpu) ([original paper](https://www.microsoft.com/en-us/research/wp-content/uploads/2005/01/p1000-loop.pdf), [presentation](https://www.youtube.com/watch?v=2OAPHn_YWGA)).

**There is a patent (US7564459B2) on using this for cubic curves until 2026...**
**So for now this only does quadratic curves, which is enough for true type fonts and can be used to approximate cubic ones.**

### Features:
* Cubic to quadratic curve simplification (Works surprisingly well, but the error is quite big with the simple approach I implemented)
* Curve intersection removal by finding intersection points and generating new curves to that point.
* Curve overlap reduction by subdividing overlapping curves.
* Curve simplification by merging points that are close and reducing the order of a curve if it's higher than needed.
* Brute force polygon triangulation that mostly just works. It's slow and can run into precision issues, but other than that it can handle a lot. (I've looked at different libraries to use, but they are either only free for non commercial projects or have other restrictions. I am considering to eventually try poly2tri if my solution ends up too slow).
* Loop-Blinn based quadratic curves to triangles mesh generation.
