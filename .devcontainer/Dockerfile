FROM espressif/idf:v4.4

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends locales \
    && locale-gen en_US.UTF-8 \
    && echo "source /opt/esp/idf/export.sh" >> /root/.bashrc

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8
