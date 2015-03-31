FROM ggrandes/ubuntu32:12.04
RUN adduser --disabled-password --shell /bin/bash --gecos '' suturo && \
	mkdir -p /home/suturo/.rviz /home/suturo/.cache /data/db && \
	chown suturo:suturo -R /home/suturo/.rviz /home/suturo/.cache /data/db
VOLUME /home/suturo/.rviz /home/suturo/.cache /data/db
USER suturo