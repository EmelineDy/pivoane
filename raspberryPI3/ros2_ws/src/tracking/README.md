# Tracking node

The tracking node is the link between the AI detection and the car behavior.


## Usage

The node receives continuously a size 2 array (close_bounding_boxes) from the AI detection. 

close_bounding_boxes[0] contains the id from the closest road sign, while close_bounding_boxes[1] contains the closest pedestrian. 
If no sign and/or pedestrian are detected, the id received is "Empty".

When a sign is detected, we increase a counter specific to the sign, and decrease the others road signs while giving a max value and min value.
Once a counter value goes past a fixed threshold, the node believes the road sign is really here, and its not a noise or false detection.

When the same counter start decreasing drastically, it implies the car has driven past the road sign, meaning it has to react. To start the reaction, the node send a message Reaction containing the sign type.
