## Face recognition

For our third task we were training a classifier to recognize twenty-one different faces, one face of Gargamel and twenty faces of women. We divided this task into three parts.

1. Obtaining training data

2. Training different classifiers

3. Evaluating the performance of trained classifiers

### 1. Preparing training data

This was the most time consuming part in this task. We had to capture images of faces, crop them and then convert them into embeddings for training our classifiers.

#### 1.1 Capturing images of faces

For training the classifiers we decided to capture two sets of photos, to best replicate the conditions in the real world and in the gazebo simulation:

1. The first set of images was obtained with a physical camera. We printed the faces on A4 sheets of paper and captured photos of them under different lighting conditions and viewing angles. We obtained around 220 images of this kind.

2. The second set of face images was obtained in the gazebo simulation. We drove the robot around the map, detected faces on the walls and saved them as images. We obtained around 950 images of this kind.

Here are the examples of our captured images.
[insert images!!!]

#### 1.2 Preprocessing images

This step was relatively easy. We first detected the faces with our face detector, that we created for homework1. Then we cropped the images to obtain faces.

Note: This first step of detecting and cropping the faces was not necessary for the second set of our images, i.e. the ones taken in the gazebo simulator. That is because our pipeline for detecting faces in ros already includes cropping the faces out of the image. All we had to do in this case was to just save the photos and label them.

We then fed the images to a python module named `face_recognition` to obtain embedding, these are actually 128 dimensional vectors, that represent our faces. We saved these embeddings into csv files along with labels of the faces for each embedding.

The main reason for using this model over others is, that it is very straightforward and easy to use. It enables us to load the images and create embeddings in just a few lines. The second reason being, that the model is very powerful and accurate, since in the background it actually uses `dlib`'s state-of-the-art face recognition built with deep learning.

Before using the embeddings for training we normalized them to increase classification accuracy mainly on knn classifiers. Normalization of features didn't have an impact on other classifiers we tested.

### 2. Face recognition

With our datasets ready, we then trained different classifiers to recognize faces and computed their accuracy. We envisioned three different approaches.

First, we trained and tested our models only on embeddings of faces that were captured in the simulation. This way we can best adjust the models to the conditions in the simulation and achieve the best classification accuracy for our task of recognizing faces. As we will see later on, this approach was very effective.

Second, we trained and tested our models only on embeddings of faces that were captured with the camera in a physical world. This approach ensured that our models could also be trained to work in rougher conditions, for example under more complex lighting setting and blur that is usually present in photographs, that is not simulated in the gazebo simulator. This approach was also very effective.

Third, we wanted to see how the models will behave if we trained them with both datasets at once. This approach ensured, that our models could accurately classify faces in the simulation, but could also perform well in more diverse conditions. Although a bit less accurate than the previous two, we can say that this approach was effective as well.

#### 2.1 Model selection

We are mainly interested in these three classifiers: knn, naive bayes and linear svm.

We chose knn because it is a standard and very simple model, that is easy to understand. We don't, however, expect it to perform particularly well on this task because of high dimensionality of our data.

The second model, naive bayes is also a standard model. It is very fast and easy to predict new the class of new data and it performs well in multi-class prediction. That is why we expect it to perform well.

The third model that we are interested in is support vector machine. This is a standard and very accurate and robust classifier that is often used in smaller datasets where deep learning approaches aren't feasible. It also performs very well on large feature spaces, that is why we expect this classifier to be very effective for our task.

We have also included some other classifiers such as decision tree and random forest to see how well they perform, out of curiosity.

