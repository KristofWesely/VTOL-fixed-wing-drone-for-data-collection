# VTOL-fixed-wing-drone-for-data-collection
This repository presents the development of a proof of concept Vertical Takeoff and Landing (VTOL) fixed-wing UAV

The `Calculations.ipynb` Jupyter notebook contains some estimations of thrust, and power consumption estimations as well as calculations for Ground Sampling Distance given a camera and mission attributes.

The Flight logs folder contians the vehicle logs recorded during the testing session

The parts folder contains all the parts designed and used during the thesis. The parts of E-VTOL-1 by Eclipson are under copyright, therefore they are not shared.

The Software folder contains the `Tailsitter_ArduPilot_parameters.param` with the parameters for ArduPilot used by the Tailsitter prototype.

The Software folder also includes the `Data_collection.py` script which runs on a Raspberry Pi mounted on the prototype to collect aerial data periodically.

Lasly the Software/Super-Resolution folder contains `Super_resolution_of_drone_images.ipynb` Jupyter notebook for processing aerial images and applying a range of different SR models to compare. Real-ESRGAN outperformed all the other models, therefore it was chosen to be finetuned using the DSR dataset - Source: https://github.com/IVRL/DSR