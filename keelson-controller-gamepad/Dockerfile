FROM ghcr.io/rise-maritime/porla:v0.4.1

COPY requirements.txt requirements.txt

RUN pip3 install --no-cache-dir -r requirements.txt

COPY --chmod=555 ./bin/* /usr/local/bin/
