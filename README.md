# GRSFramework

**Granular Rigid Body Simulation Framework**: efficient research tools to simulate **non-smooth** granular rigid body dynamics.

![BuildStatus](https://app.travis-ci.com/gabyx/ExecutionGraph.svg?branch=master)

![C++](https://img.shields.io/badge/c%2B%2B-11/14-green.svg)
![Deps](https://img.shields.io/badge/dependencies-Ogre%7Cboost%7C%20Assimp%7CMPI%7CHDF5%7Ceigen3%7Cpugixml%7Cmeta%7CApproxMVBB%7CCudaFramework%7Cpython3-blue.svg)
![System](https://img.shields.io/badge/system-linux,osx,%7Bwindows%7D-lightgrey.svg)

[Homepage](http://gabyx.github.io/GRSFramework/)
[Wiki](https://github.com/gabyx/GRSFramework/wiki)

# Videos

<table>
<tr>
<td width="50%">
   <a href="https://player.vimeo.com/video/160352926?title=0&byline=0"><img src="https://raw.githubusercontent.com/gabyx/GRSFramework/gh-pages/videos/Avalanche-P-9Tracking.png" width="100%"></a>
    <span>Coloring: Velocity</span>
</td>
<td width="50%">
   <a href="https://player.vimeo.com/video/160352927?title=0&byline=0"><img src="https://raw.githubusercontent.com/gabyx/GRSFramework/gh-pages/videos/Avalanche-P-9StaticFarProcs.png" width="100%"></a>
    <span>Coloring: Process Domain</span>
</td>
</tr>
<tr>
<td width="50%">
   <a href="https://player.vimeo.com/video/160352928?title=0&byline=0"><img src="https://raw.githubusercontent.com/gabyx/GRSFramework/gh-pages/videos/Avalanche-P-9CloseUpColored.png" width="100%"></a>
    <span>Coloring: Random</span>
</td>
<td width="50%">
   <a href="https://player.vimeo.com/video/160352925?title=0&byline=0"><img src="https://raw.githubusercontent.com/gabyx/GRSFramework/gh-pages/videos/SimVsCIV-VelocityMag-P-9.png" width="100%"></a>
    <span>Coloring: Velocity</span>
</td>
</tr>
</table>

**Chute flow simulation & experiment with 1 million spheres:**

- _method:_ Moreau time-stepping with unilateral contacts and Coulomb friction
- computed on 384 cores with `GRSFSimMPI` in 12 h, rendered with `GRSFConverter` and `prman` in 24 h.
- _time step:_ 0.0002 s, _friction coefficient:_ 0.8
- _restitution coefficient:_ 0.0 (fully inelastic impacts)
- _global contact iterations:_ 1000
