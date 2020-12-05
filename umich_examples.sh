#!/bin/bash
pathDatasetUmich='/Datasets/umich' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular Examples
echo "Launching DH01 with Monocular sensor"
./Examples/Monocular-Odometric/mono_umich ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Odometric/umich.yaml "$pathDatasetUmich"/DH01
    ./Examples/Monocular-Odometric/umich_TimeStamps/DH01.txt dataset-DH01_mono

# Monocular-Inertial Examples
echo "Launching DH01 with Monocular-Inertial sensor"
./Examples/Monocular-Inertial/mono_inertial_umich ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Odometric/umich.yaml "$pathDatasetEuroc"/DH01 ./Examples/Monocular-Odometric/umich_TimeStamps/DH01.txt dataset-DH01_monoi


# Monocular-Odometric Examples
echo "Launching DH01 with Monocular-Odometric sensor"
./Examples/Monocular-Inertial/mono_odometric_umich ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Odometric/umich.yaml "$pathDatasetEuroc"/DH01 ./Examples/Monocular-Odometric/umich_TimeStamps/DH01.txt dataset-DH01_monoi
