FROM intel4coro/base-notebook:20.04-noetic

# Install SWI Prolog
USER root
RUN apt install -y software-properties-common && \
  apt-add-repository -y ppa:swi-prolog/stable
RUN apt update && \
  apt install -y swi-prolog
ENV SWI_HOME_DIR=/usr/lib/swi-prolog

# Install MongoDB Community Edition
ARG MONGODEB_VERSION=7.0
RUN curl -fsSL https://www.mongodb.org/static/pgp/server-${MONGODEB_VERSION}.asc | \
    gpg -o /usr/share/keyrings/mongodb-server-${MONGODEB_VERSION}.gpg --dearmor
RUN echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-${MONGODEB_VERSION}.gpg ] https://repo.mongodb.org/apt/ubuntu focal/mongodb-org/${MONGODEB_VERSION} multiverse" | \
    tee /etc/apt/sources.list.d/mongodb-org-${MONGODEB_VERSION}.list
RUN apt update && apt install -y mongodb-org
RUN mkdir -p /data/db && \
    chown -R ${NB_USER}:users /data/db

# Install Knowrob
USER ${NB_USER}
ENV KNOWROB_WS=${HOME}/workspace/knowrob
RUN mkdir -p ${KNOWROB_WS}/src
WORKDIR ${KNOWROB_WS}/src
RUN wstool init && \
    wstool merge https://raw.github.com/knowrob/knowrob/master/rosinstall/knowrob-base.rosinstall && \
    wstool update

# Switch to root user to install ROS dependencies
USER root
RUN rosdep update && rosdep install -y -r --ignore-src --from-paths . && \
    rosdep fix-permissions

COPY --chown=${NB_USER}:users knowrob_cloud ${KNOWROB_WS}/src/knowrob_cloud
COPY --chown=${NB_USER}:users prolog /prolog

# Build catkin workspace
USER ${NB_USER}
WORKDIR ${KNOWROB_WS}
RUN source ${ROS_PATH}/setup.bash && catkin_make

# Source knowrob workspace on a new bash shell
RUN echo "source ${KNOWROB_WS}/devel/setup.bash" >> ${HOME}/.bashrc

# Install jupyter knowrob kernel
USER ${NB_USER}
COPY --chown=${NB_USER}:users . ${HOME}/jupyter-knowrob
WORKDIR ${HOME}/jupyter-knowrob
RUN pip install .
COPY --chown=${NB_USER}:users ./kernel.json ${HOME}/.local/share/jupyter/kernels/jknowrob/kernel.json

# Download the example_neem
USER root
RUN mkdir /neem_data && chown ${NB_USER}:users /neem_data
USER ${NB_USER}
RUN cd /neem_data && \
    ln -s /neem_data ${HOME}/jupyter-knowrob/lectures/data && \
    wget https://seafile.zfn.uni-bremen.de/f/b66a71b257084d459b6c/?dl=1 -O example_neem.zip && \
    unzip example_neem.zip && \
    mv example_neem/* ./ && \
    rm -r example_neem example_neem.zip

# jupyterlab interface configuration
COPY --chown=${NB_USER}:users binder/jupyter-settings.json /opt/conda/share/jupyter/lab/settings/overrides.json
COPY --chown=${NB_USER}:users binder/webapps.json ${ROS_WS}/src/rvizweb/webapps/app.json

# Entrypoint starting knowrob and mongodb
COPY --chown=${NB_USER}:users binder/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

WORKDIR ${HOME}/jupyter-knowrob/lectures
RUN ln -s ${KNOWROB_WS} ${PWD}/KNOWROB_WS
