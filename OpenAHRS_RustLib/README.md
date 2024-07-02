# OpenAHRS_RustLib

# Overview
OpenAHRS is an open-source project that provides a collection of functions for Attitude and Heading Reference System (AHRS) algorithms. The project is written in Rust and provides functionalities for quaternion manipulation, matrix operations, and raw sensor measurements processing.

# Table of Contents
- License
- Dependencies
- Features
- Usage
- Contributing
- Authors
- Resources
- Acknowledgments

# Licence
This project is licensed under the AGPL-3.0 license.

# Dependencies
The project has minimal dependencies, and it is designed to be easily integrated into other projects. The code uses libm library and should be compatible with various platforms and no_std environement.

# Features

Quaternion manipulation and operations:
- Initialization
- Conjugation
- Normalization
- Multiplication
- Conversion to rotation matrix
- Conversion to DCM (Direct Cosine Matrix)
- Conversion to Tait-Bryan Angles
- And more..

Matrix manipulation and operations:
- Initialization
- Identity matrix initialization
- Copying
- Addition
- Subtraction
- Multiplication
- Transposition
- Inversion
- Eigenvalues and Eigenvectors
- And more..

Sensor Measurements:
- Structures for raw gyroscope, accelerometer, and magnetometer measurements

Various filter, sensor fusion and orientation/attitude estimator algorithms:
- AR (Angular Rate) based on Taylor series interpolation (Tested)
- AQUA - Algebraic Quaternion Algorithm (Tested)
- Davenport - Davenport's Q Method (Tested)
- FAMC - Fast Accelerometer-Magnetometer Combination (Tested)

Future algorithms:
- FLAE - Fast Linear Attitude Estimator
- EKF - Extended Kalman Filter
- FNAE - Fourati’s Nonlinear Attitude Estimation)
- FQA - Factored Quaternion Algorithm
- MOF - Madgwick's Orientation Filter
- MOF2 - Mahony's Orientation Filter
- OLEQ - Optimal Linear Estimator of Quaternion
- QUEST - Quaternion Estimator
- MEKF - Multiplicative Extended Kalman Filter
- IEKF - Invariant Extended Kalman Filter
- EF - Equivariant Filter
- PF - Particle Filter (Euler angles, DCM and quaternion)
- VQF - Versatile Quaternion-based Filter

# Usage
To use OpenAHRS in your project, include the necessary crates in your source code. For more details on function usage, refer to the individual crate files in the source code.

# Contributing
Contributions are welcome! If you would like to contribute to the project, please contact me.

# Author
I'm Titouan BORENSZTEJN (french mechatronic engineer). You can contact me for any information about this project at <borensztejn.titouan@gmail.com> or <openrtvc@gmail.com>

# Resources
All the scientific and technical documentation and other resources are available with the project. However, some errors are hidden in the documents, often due to a lack of proofreading. The errors I have found are corrected in the PDF files. If you find any others, please share them. Thank you for your contribution.

# Acknowledgments
Special thanks to the open-source community and contributors for their valuable contributions. This code has been developed in partnership with the French company FBO (Forges de Belles Ondes at <http://www.snfbo.com/>) and gives them the right to commercially exploit the software, but they must make any changes made to this part of the code public.
