# Exported dataset for Jorge Marqués García / JMarques-project-1

To import this data into a new Edge Impulse project, either use:

* The Edge Impulse CLI (https://docs.edgeimpulse.com/docs/edge-impulse-cli/cli-overview), run with:

    edge-impulse-uploader --clean --info-file info.labels

* Or, via the Edge Impulse Studio. Go to **Data acquisition > Upload data**.


All the data recolected to make the maching learning impulse that make movement recognition in the three axis, based on the acceleromenter included in the board. 
Due to the requierements movement will be divided in 6:
 - Positive Roll
 - Negative Roll
 - Positive Pitch
 - Negative Pitch
 - Positive Yaw
 - Negative Yaw  

Note that, there are three different patterns and then each one of them in positive and negative orientation.
For the baseline noise, initially, two types of noise has been recolected: 
    - Noise => with the BLE board leaning on the table
    - NoiseUP => Holding the BLE board the same high from the table like making the acquisition but without movement. 

At the end, only NoiseUP data has been used for tranning and testing the model, because with both the model performs worst. 

The data has been taken in intervals of 9 minutes and 20 seconds (maximum allowed by the platform) at 20Hz os sampling. 
Making the same movement in each set in both orientations (positive and negative). To achieve most accuracy in each movement, 
a delay time of 4 seconds between each change of orientation has been implemented. 