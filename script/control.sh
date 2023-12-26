#!/bin/bash

# Lower PID values for smoother motion
P=7.0  # Significantly increased Proportional Gain for more torque
D=1.2  # Higher Derivative Gain for better control of movement
I=0.02 # Slightly higher Integral Gain to address any steady-state error

# Set target positions
HAA_POS=0.05
HFE_POS=-0.4
KFE_FR_POS=1.0
KFE_HR_POS=1.2
ANKLE_POS=0.3

# Apply joint positions
gz joint -m solo -j FL_HAA --pos-t $HAA_POS --pos-p $P --pos-i $I --pos-d $D
gz joint -m solo -j FR_HAA --pos-t $HAA_POS --pos-p $P --pos-i $I --pos-d $D
gz joint -m solo -j HL_HAA --pos-t $HAA_POS --pos-p $P --pos-i $I --pos-d $D
gz joint -m solo -j HR_HAA --pos-t $HAA_POS --pos-p $P --pos-i $I --pos-d $D

gz joint -m solo -j FL_HFE --pos-t $HFE_POS --pos-p $P --pos-i $I --pos-d $D
gz joint -m solo -j FR_HFE --pos-t $HFE_POS --pos-p $P --pos-i $I --pos-d $D
gz joint -m solo -j HL_HFE --pos-t $HFE_POS --pos-p $P --pos-i $I --pos-d $D
gz joint -m solo -j HR_HFE --pos-t $HFE_POS --pos-p $P --pos-i $I --pos-d $D

gz joint -m solo -j FL_KFE --pos-t $KFE_FR_POS --pos-p $P --pos-i $I --pos-d $D
gz joint -m solo -j FR_KFE --pos-t $KFE_FR_POS --pos-p $P --pos-i $I --pos-d $D
gz joint -m solo -j HL_KFE --pos-t $KFE_HR_POS --pos-p $P --pos-i $I --pos-d $D
gz joint -m solo -j HR_KFE --pos-t $KFE_HR_POS --pos-p $P --pos-i $I --pos-d $D


