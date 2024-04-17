# Configuration INFO

## Variable Names

There is a mismatch between code variables names and paper ones. This is a correspondance list:

| Paper |   Code   |
|-------|----------|
|  cX1  |   cX1    |
|  cX4  |   cX2    |
|  cX5  |   cX3    |
|  cN1  |   cN1    |
|  cN5  |   cN2    |
|  cN7  |   cN3    |
|  cN2  |  cN1neg  |
|  cN6  |  cN2neg  |
|  cN8  |  cN3neg  |
|  b1++ |  b1pos   |
|


## Values meaning

### `cX#, cY#, cN#`
The *c* parameters are evaluated using the identification procedure.

### `lambdaPos, lambdaNeg`
Using the MotorPercentages and RPM data points recorded in the experiments to find a best linear fit parameter that converts percentages to RPM. The *Pos* and *Neg* values refer to the propellers rotating clockwise (forward) or anti-clockwise (backward). The values are found for both the Port and Starboard motors and the values is then averaged.
The data points used are the one extracted from the December2024 and January2024 DELFIM tests in Lisbon, and are collected in the relative `delfim_model_identification` folder. The code used to calculate these variables is the MATLAB script `thrusters_params_bestfit.m` ([link](https://unigeit.sharepoint.com/:u:/r/sites/GRAALDIBRIS/Documenti%20condivisi/GRAAL%20Drive/Research/ASV%20Modeling/delfim_model_identification/functions/thrusters_params_bestfit.m?csf=1&web=1&e=ngaHHM)).

### `b#Pos, b#Neg`
The *b* values are the parameters of the thruster model and relate the RPMs to the generated mechanical force. b1++ is not identifiable, and the value is calculated from the datasheet of the maximum static thrust at maximum RPM using the formula derived from *F = b1\*n^2*:

b1++ = F_max/(n_max ²)

which for the DELFIM is:

b1++ = 151.1 / (330 ²) = 0.0013875

With the values taken from experimental data provided by IST ([link](https://unigeit.sharepoint.com/:x:/r/sites/GRAALDIBRIS/Documenti%20condivisi/GRAAL%20Drive/Research/ASV%20Modeling/delfim_model_identification/misc/Delfim%20Thrust%20Curve.xlsx?d=w738b313996c0403d89887f60f728a8cd&csf=1&web=1&e=I0s9gI)). 
The remaining values are the result of the identification process.

### `kPos, kNeg`
They model the transverse thrust forces. Their values are in the results of the identification process.

### `rpmDyn***`


These parameters contain the thruster dynamics (RPM) of the catamaran engines (inserted in the C++ code of the model and navigation filter) with the formula:

```
double rpm_gain_s = (n_s < 0) ? params.rpmDynNegPerc : params.rpmDynPosPerc;
n_p = params.rpmDynState * n_p + rpm_gain_p * h_p;
```

The values are taken using the following simple first order system in MATLAB:

```
G = 13.8/(1+s*0.35)  # Pos
c2d(G,0.01,'zoh')
G = 10.1/(1+s*0.35)  # Neg
c2d(G,0.01,'zoh')
```



*N.B. tau=0.35 found by trial and error, to be explored further*.

rpmDynState: 0.9718;
rpmDynPosPerc: 0.1056;
rpmDynNegPerc: 0.156;