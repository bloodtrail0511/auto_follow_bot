hyp改tiny
img-size改成416
python .\train.py --cache-images --device 0 --name car_v12 --epochs 200 --hyp data/hyp.scratch.tiny.yaml --img-size 416 416