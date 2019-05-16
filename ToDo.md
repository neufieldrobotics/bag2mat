# auto detect ns of topics in rosbag #


A scenario where automated script is run to convert rosbags from multiple vehicles with different ns

A single ros bag on host pc recording topics from multiple robots with different ns for same topic type


Instead of script changing topics config file. the bag2matpy auto detects.


### how to do it##


a=rosbag.Bag('/home/mithun/Desktop/uas1_2019-05-09-11-38-08.bag')
b=a.get_type_and_topic_info().topics.keys()
for z in b:
    if z.find('/simplenav/odom') > 0:
        print z
        # do rsplit and get ns here and pass that to variable name and call savetopics2mat func
        # we do this only once and generate final list of topics, instead of adding ns before topic name

## add ns to start_time  ##
    

   In above scenario adding ns to start_time doesn't make sense

## when dict becomes huge in future with all msg types ##


load dicts specific to pkg like std_msgs if Int32 during runtime once per topic 