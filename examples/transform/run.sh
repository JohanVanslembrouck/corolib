#!/bin/sh

echo Running ./tr-p0100trf
./tr-p0100trf

echo Running ./tr-p0200
./tr-p0200
echo Running ./tr-p0200lb
./tr-p0200lb
echo Running ./tr-p0200trf
./tr-p0200trf

echo Running ./tr-p0202
./tr-p0202
echo Running ./tr-p0202lb
./tr-p0202lb
echo Running ./tr-p0202trf
./tr-p0202trf

echo Running ./tr-p0204
./tr-p0204
echo Running ./tr-p0204lb
./tr-p0204lb
echo Running ./tr-p0204trf
./tr-p0204trf

echo Running ./tr-p0206
./tr-p0206
echo Running ./tr-p0206lb
./tr-p0206lb
echo Running ./tr-p0206trf
./tr-p0206trf
