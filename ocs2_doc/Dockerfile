FROM ros:noetic 

RUN apt-get update \
  && apt-get upgrade -y \ 
  && apt-get install -y \
    doxygen \
    python3-pip \ 
    git \
    rsync \
    wget \
    curl 

RUN pip3 install \
  sphinx \
  breathe \
  sphinx-rtd-theme \
  sphinxcontrib.bibtex
