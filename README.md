# Registration

## DEVELOPER
![pic](./ROBOT-GRASP/example/IDLER.png)

## Procedure
This program consists of four parts: Segmentation, classification, registration and reflection. First, depth data can be divided into serval regions with improved region-growing segmentation. Then classifly region catergory with a classic linear svm with HOG as feature.
### Segmentation

### Classification

### Registration

### Reflection

## Example
### 


### Result
![Example](./ROBOT-GRASP/example/Example.png)
![Point-Cloud](./ROBOT-GRASP/example/Point-Cloud.gif)

### Interpretion

## ERROR & SOLUTION
### Error 1 
```
error LNK2038: mismatch detected for 'RuntimeLibrary'
```
### SOLUTION
```
c/c++ -> code generate -> replace MT to MD 
```