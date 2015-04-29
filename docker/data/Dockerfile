FROM ggrandes/ubuntu32:12.04
RUN adduser --disabled-password --shell /bin/bash --gecos '' suturo && \
	mkdir -p /persistent && \
	chown suturo:suturo -R /persistent
VOLUME /persistent
USER suturo