## Description

MSmooth is a spatial smoother that doesn't touch edges.

This plugin is [a port of the VapourSynth plugin MSmooth](https://github.com/dubhater/vapoursynth-msmoosh).

### Requirements:

- AviSynth 2.60 / AviSynth+ 3.4 or later

- Microsoft VisualC++ Redistributable Package 2022 (can be downloaded from [here](https://github.com/abbodi1406/vcredist/releases))

### Usage:

```
vsMSmooth (clip, float "threshold", float "strength", bool "mask", bool "luma", bool "chroma")
```

### Parameters:

- clip\
    Clip to process. It must be in 8..16-bit planar format.
    
- threshold\
    Sensitivity of the edge detection. Decrease if important edges are getting blurred. This parameter became a percentage in order to make it independent of the bit depth.\
    Must be between 0.0 and 100.0.\
    Default: 6.0.
            
- strength\
    Number of times the image should be blurred. Increase to make the image smoother and the filter slower.\
    Must be between 0.0 and 25.0.\
    Default: 3.0.
    
- mask\
    If True, the edge mask will be returned instead of the filtered frames.\
    Default: False.
    
- luma, chroma\
    Planes to process.\
    When mask=True, the untouched planes will contain garbage.\
    When mask=False, the untouched planes will be copied.\
    When the clip is RGB, all planes are processed.\
    Default: luma = true; chroma = false
