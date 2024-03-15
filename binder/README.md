# Jupyter Knowrob on BinderHub 

[![Binder](https://binder.intel4coro.de/badge_logo.svg)](https://binder.intel4coro.de/v2/gh/yxzhan/jupyter-knowrob.git/master?labpath=lectures%2Ftut2-semweb.ipynb)

Running [Knowrob](https://www.knowrob.org/) with Jupyter notebook on Binderhub.

## Learn more about Knowrob

- Home: https://www.knowrob.org/
- Docs: https://knowrob.github.io/knowrob/
- Publications: https://www.knowrob.org/publications
- Github: https://github.com/knowrob/knowrob
- Jupyter extension: https://github.com/sasjonge/jupyter-knowrob
- Notebooks: https://github.com/sasjonge/jupyter-knowrob/tree/master/lectures

## Development

### Test Image Locally (Under repo directory)

- Run Docker image with X-forwarding

  ```bash
  chmod -R g+w ./ && export GID=$(id -g) && \
  docker compose -f ./binder/docker-compose.yml up --build
  ```

- Open Web browser and go to http://localhost:8888

### To stop and remove container

```bash
docker compose -f ./binder/docker-compose.yml down
```
  
### To force docker image rebuild from scratch

```bash
chmod -R g+w ./ && export GID=$(id -g) && \
docker compose -f ./binder/docker-compose.yml up --build --no-cache
```
