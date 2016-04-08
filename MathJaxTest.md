## Simulating Non-Smooth Rigid Body Dynamics
The interplay between the simulators ``GRSFSim``, ``GRSFSimMPI``, ``GRSFGUI`` and the converter ``GRSFConverter`` of the framework is best described by the following diagram:
<p align="center">  
<img src="https://rawgit.com/wiki/gabyx/GRSFramework/images/workflow.svg" alt="GRSF Workflow" width="700px"/>
</p>
The scene consisting of rigid bodies to simulate as well as solver settings, recorder settings, visualization settings etc. is described by a scene XML file. The scene file XML is described by an XSD scheme which is located in ``simulations/sceneFileValidation`` which serves for scene file validation as well as reference.
More information about the scene file can be found here [SceneFile](https://github.com/gabyx/GRSFramework/wiki/SceneFile).    
 
The output of the simulator tools ``GRSFSim``, ``GRSFSimMPI``, ``GRSFGUI`` is a binary file [``.sim``](https://github.com/gabyx/GRSFramework/wiki/MultiBodySimFile) containing for each body a generalized displacement
`\( \mathbf{q} =  [{_\mathrm{I}}\mathbf{r}_{S}^\top \ , \ \mathbf{p}_{\mathrm{KI}}^\top]^\top \in \mathbb{R}^7 \)` and a generalized velocity `\( \mathbf{u} =  [{_\mathrm{I}}\dot{\mathbf{r}}_{S}^\top \ , \ {_\mathrm{K}}\boldsymbol{\Omega}^\top]^\top \in \mathbb{R}^6 \)`. 

The vector `\({_\mathrm{I}}\mathbf{r}_{S} \in \mathbb{R}^3\)` is the position of the center of gravity `\(S\)` represented in the inertial coordinate system `\(\mathrm{I}\)` and `\(\mathbf{p}_{\mathrm{KI}} \in \mathbb{R}^4\)` is the quaternion describing a rotation from the inertial coordinate system `\(\mathrm{I}\)` to the body-fixed coordinate system `\(\mathrm{K}\)`. 

The term `\({_\mathrm{I}}\dot{\mathbf{r}}_{S}\)` is the absolute velocity of the center of gravity `\(S\)` of the body represented in the inertial coordinate system `\(\mathrm{I}\)` and `\({_\mathrm{K}}\boldsymbol{\Omega} = {_\mathrm{K}}\boldsymbol{\omega}_{\mathrm{IK}} = \mathbf{A}_{\mathrm{IK}}^\top \dot{\mathbf{A}}_{\mathrm{IK}}\)` is the angular velocity of the body represented in body-fixed coordinate system `\(\mathrm{K}\)`. 

