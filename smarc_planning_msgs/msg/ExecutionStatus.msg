# if true then the head of the execution_queue is currently being executed.
std_msgs/Header header
bool currently_executing
ConditionalAction[] execution_queue
