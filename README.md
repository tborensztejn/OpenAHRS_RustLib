# OpenAHRS_RustLib

# What is an AHRS?
An AHRS (Attitude and Heading Reference System) is a system used to determine and track the orientation of an object relative to a fixed reference frame. It is crucial in various fields such as aerospace, robotics, and navigation. AHRS typically use sensors like gyroscopes, accelerometers, and magnetometers to provide accurate and reliable measurements under all conditions.

# Overview
OpenAHRS is a Rust library dedicated to AHRS algorithms, offering a comprehensive range of features for quaternion manipulation, matrix and vector operations, and processing raw sensor data. Whether you are looking to perform complex quaternion manipulations or integrate robust sensor fusion and filtering algorithms into your projects, OpenAHRS is the ideal solution.

OpenAHRS is designed to be reliable, optimized, versatile, educational, and performant. Its unique structure ensures unparalleled maintainability. All code lines are commented, and algorithms are sourced. You will find technical and scientific references for each one.

# Why This Project?
This field is highly complex (abstract theoretical foundations and complex mathematics) and is among the most active in technological research. In the industry, it is a highly competitive field, though not widely known, as it partly contributes to guidance systems (rockets, airplanes, missiles, satellites, etc.). There are very few open-source projects on this subject with a genuine desire to share domain knowledge, even fewer in the Francophone world. This project contributes to the democratization of knowledge and offers a valuable resource for those interested in this complex, vital, and under-recognized field.

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

Quaternion Manipulation
- Basic Operations: Initialization, conjugation, normalization, addition, subtraction, inversion, multiplication, etc.
- Advanced Operations: Conversion to rotation matrix (Direct Cosine Matrix - DCM), Axis-Angle conversion, Tait-Bryan Angles (Euler) conversion, and more...

Matrix and Vector Manipulation (Optimized Linear Algebra Algorithms)
- Basic Operations: Initialization, identity matrix, copy, addition, subtraction, multiplication, transposition, inversion.
- Advanced Operations: Eigenvalue and eigenvector computation, QR decomposition, and more...

Complex Numbers
- Fast Fourier Transform and Inverse (FFT & IFFT)
- FIR and IIR Filters

Sensor Management
- Sensor parameterization and calibration (3D gyroscope, accelerometer, and magnetometer)
- Fault detection and more...

Filtering, Estimation, and Various Data Fusion Algorithms
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
To integrate OpenAHRS into your project, simply include the necessary crates in your source code. For more details on function usage, refer to the crate files in the source code.

# How to Contribute?
Your help is invaluable! If you wish to contribute to the project, whether by finding bugs, improving documentation, or adding new features, don’t hesitate to contact me. Your contributions will enhance OpenAHRS and support community progress.

Your expertise can make a significant difference in the advancement of this project. Every contribution matters and is greatly appreciated!

# Author
I'm Titouan BORENSZTEJN (french mechatronic engineer). You can contact me for any information about this project at <borensztejn.titouan@gmail.com> or <openrtvc@gmail.com>

# Resources
All the scientific and technical documentation and other resources are available with the project. However, some errors are hidden in the documents, often due to a lack of proofreading. The errors I have found are corrected in the PDF files. If you find any others, please share them. Thank you for your contribution.

# Acknowledgments
Special thanks to the open-source community and contributors for their valuable contributions. This code has been developed in partnership with the French company FBO (Forges de Belles Ondes at <http://www.snfbo.com/>) and gives them the right to commercially exploit the software, but they must make any changes made to this part of the code public.

# Join Us on Patreon!
If you appreciate this project and wish to support its development, join us on Patreon and Github.

Your support will allow us to continue improving OpenAHRS and bringing even more powerful features to the community.

Thank you for your support and interest in OpenAHRS! Together, we can push the boundaries of AHRS technology.

If you have any questions or feedback, I would be delighted to discuss them with you. 🚀

Join us and explore the possibilities that OpenAHRS offers!

Contributions are welcome! If you wish to participate or have suggestions, feel free to contact me.
