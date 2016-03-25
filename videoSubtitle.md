**chute flow simulation & experiment with 1 million spheres**   
*method:* Moreau time-stepping with unilateral contacts and Coulomb friction 
computed on 384 cores with ``GRSFSimMPI`` in 12 h, rendered with ``GRSFConverter`` and ``prman`` in 24 h.   
*time step:* 0.0002 s, *friction coefficient:* 0.8
*restitution coefficient:* 0.0 (fully inelastic impacts)    
*global contact iterations:* 1000    
*coloring:* velocity magnitude / process domain