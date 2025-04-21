FROM archlinux

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies using pacman
RUN pacman -Syu --noconfirm \
    base-devel cmake git wget \
    eigen boost irrlicht glm jsoncpp \
    unzip zip \
    libxxf86vm libxrandr libxi libxinerama libxcursor \
    libx11 \
    mesa glu \
    libpng libjpeg-turbo bzip2 freeglut \
    asio openmpi nlohmann-json bullet websocketpp && \
    pacman -Scc --noconfirm

# Download Chrono source to a dedicated directory and build in a separate build directory
WORKDIR /opt
RUN git clone https://github.com/projectchrono/chrono.git chrono-src
RUN cd chrono-src && \
    git checkout release/9.0 && \
    mkdir /opt/chrono-build && \
    cd /opt/chrono-build && \
    cmake ../chrono-src \
      -DCMAKE_BUILD_TYPE=Release \
      -DENABLE_MODULE_IRRLICHT=ON \
      -DENABLE_MODULE_VEHICLE=ON && \
    make -j$(nproc) && make install

# Copy project files
WORKDIR /app
COPY . /app

# Build your project: remove any existing build folder to avoid CMakeCache conflicts
RUN rm -rf build && mkdir -p build && cd build && \
    cmake .. && \
    make -j$(nproc)

# Expose port 17863 for TCPPositionServer and 9090 for ROSBridge
EXPOSE 17863
EXPOSE 9090

# Set display for GUI
ENV DISPLAY=:0

# Default command to run your program (edit as needed)
WORKDIR /app/build
CMD ["./main", "--no-vis"]
