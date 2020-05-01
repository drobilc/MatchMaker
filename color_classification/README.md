# Color classification

## Installation
To install, run the following commands.

```bash
python -m pip install scikit-learn==0.19
pip install colormath --no-cache-dir --no-dependencies
pip install 'networkx==2.2'
```

You must also build new message types, so execute `catkin_make` after.

## Usage
The example below shows us how to use this service in Python with rospy.

```python
import rospy
from color_classification.srv import ColorClassification, ColorClassificationResponse
from std_msgs.msg import ColorRGBA

rospy.wait_for_service('color_classifier')
classify_color = rospy.ServiceProxy('color_classifier', ColorClassification)

red = ColorRGBA(255, 0, 0, 0)
response = classify_color(red)
print(response.classified_color)
```