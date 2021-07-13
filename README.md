# edge_impulse_thunderboard_bg22_test

## Intro

This is a minimal test example of integrating the edge-impulse-sdk C/C++ library with Silicon Labs' Simplicity Studio v5. This project targets the BG22 Thunderboard and any Edge Impulse models based on accelerometer data.

This project is currently a work in progress with minimal functional testing only.

## How to use

Download the latest ```edge_impulse_example.sls``` from the releases tab, and then from Simplicity Studio v5 navigate to ```File->Import->Browse``` to select the project.

To switch out the default model for your own, overwrite ```edge-impulse-sdk/```, ```model-parameters/```, and ```tflite-model/``` with folders of the same name from a generated Edge Impulse C/C++ library.


