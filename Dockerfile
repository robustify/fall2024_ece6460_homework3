FROM robustify101/ros:homework-checker
COPY . /home/ros/src

RUN rosdep install --from-paths /home/ros/src --ignore-src -r

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
