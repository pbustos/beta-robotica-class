# person_tracker
Intro to component here

Install the following dependencies:
```
sudo apt-get install libopencv-dev
sudo apt-get install libmlpack-dev
sudo apt-get install libarmadillo-dev
sudo apt-get install libboost-all-dev
sudo apt-get install libgomp1
```


## Configuration parameters
As any other component, *person_tracker* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
EXAMPLE HERE
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <person_tracker's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/person_tracker config
```
