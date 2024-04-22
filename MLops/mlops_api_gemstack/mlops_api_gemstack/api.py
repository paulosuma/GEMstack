import requests
import click
import os
import subprocess
import signal
import sys
from tqdm import tqdm
from pprint import pprint
from urllib.parse import unquote

base_url='https://highbayapi.Nico-nico-nii.com'
save_directory = os.getcwd()

@click.group()
def cli():
    pass

@cli.command()
def list_models_info():
        response = requests.get(f"{base_url}/models")
        if response.status_code == 200:
            list_model = response.json()
            click.echo("List of models avaliable on server:\n")
            for info in list_model:
                pprint(info)
                click.echo("\n")
            return list_model
        else:
            click.echo(f"Failed to list models. Status code: {response.status_code}")

@cli.command()
@click.argument('model_id')
def model_info(model_id):
    response = requests.get(f"{base_url}/models/{model_id}")
    if response.status_code == 200:
        model_info = response.json()
        pprint(model_info)
        return model_info
    elif response.status_code == 404:
        click.echo(f"Error: {response.json()['error']}")
    else:
        click.echo(f"Failed to get model info. Status code: {response.status_code}")

@cli.command()
@click.argument('model_id')
@click.argument('new_description') 
def model_update(model_id, new_description):
    response = requests.put(f"{base_url}/models/{model_id}", json={"Description": new_description})
    if response.status_code == 200:
        click.echo("Model description updated successfully")
    elif response.status_code == 400:
        click.echo(f"Error: {response.json()['error']}")
    else:
        click.echo(f"Failed to update model description. Status code: {response.status_code}")

@cli.command()
@click.argument('model_id')
def model_download(model_id):
    response = requests.get(f"{base_url}/models/retrieval/{model_id}", stream=True)
    if response.status_code == 200:
        total_size_in_bytes = int(response.headers.get('content-length', 0))
        progress_bar = tqdm(total=total_size_in_bytes, unit='iB', unit_scale=True)
        content_disposition = response.headers.get('content-disposition')
        if content_disposition:
            filename = unquote(content_disposition.split('filename=')[-1].strip('"'))
            save_path = f"{save_directory}/{filename}"
        else:
            save_path = f"{save_directory}/default_filename.ext"
            click.echo("Filename not provided in the response headers, saving as default_filename.ext")
        
        with open(save_path, 'wb') as f:
            for chunk in response.iter_content(chunk_size=1024):
                progress_bar.update(len(chunk))
                if chunk:
                    f.write(chunk)
        progress_bar.close()
        click.echo(f"File downloaded successfully and saved to {save_path}")
    elif response.status_code == 404:
        click.echo(f"Error: {response.json()['error']}")
    else:
        click.echo(f"Failed to download the file. Status code: {response.status_code}")

@cli.command()
@click.argument('file_path')
def model_upload(file_path):
    try:
        with open(file_path, 'rb') as f:
            files = {'file': f}
            click.echo(f"Uploading {file_path}. Please wait and do not kill the process. This may take a while.")
            response = requests.post(f"{base_url}//models/upload", files=files)
            if response.status_code == 200:
                click.echo(f"Message: {response.json()['message']}. Model name: {response.json()['filename']}")
            elif response.status_code == 400:
                click.echo(f"Error: {response.json()['error']}")
            else:
                click.echo(f"Failed to upload the file. Status code: {response.status_code}")
    except FileNotFoundError:
        click.echo(f"File not found: {file_path}")
    except PermissionError:
        click.echo(f"Permission denied: {file_path}")
    except Exception as e:
        click.echo(f"An error occurred: {e}")

@cli.command()
def list_datasets_info():
    response = requests.get(f"{base_url}/datasets")
    if response.status_code == 200:
        list_model = response.json()
        click.echo("List of datasets avaliable on server:\n")
        for info in list_model:
            pprint(info)
            click.echo("\n")
        return list_model
    else:
        click.echo(f"Failed to list datasets. Status code: {response.status_code}")

@cli.command()
@click.argument('dataset_id')
def dataset_info(dataset_id):
    response = requests.get(f"{base_url}/datasets/{dataset_id}")
    if response.status_code == 200:
        dataset_info = response.json()
        pprint(dataset_info)
        return dataset_info
    elif response.status_code == 404:
        click.echo(f"Error: {response.json()['error']}")
    else:
        click.echo(f"Failed to get dataset info. Status code: {response.status_code}")

@cli.command()
@click.argument('dataset_id')
@click.argument('new_description')
@click.argument('new_source')
def dataset_update(dataset_id, new_description, new_source):

    update_data = {
        "Description": new_description,
        "Source": new_source
    }

    response = requests.put(f"{base_url}/datasets/{dataset_id}", json=update_data)
    if response.status_code == 200:
        click.echo("Dataset updated successfully")
    elif response.status_code == 400:
        click.echo(f"Error: {response.json()['error']}")
    else:
        click.echo(f"Failed to update dataset description. Status code: {response.status_code}")

@cli.command()
@click.argument('dataset_id')
def dataset_download(dataset_id):
    response = requests.get(f"{base_url}/datasets/retrieval/{dataset_id}", stream=True)
    if response.status_code == 200:
        total_size_in_bytes = int(response.headers.get('content-length', 0))
        progress_bar = tqdm(total=total_size_in_bytes, unit='iB', unit_scale=True)
        content_disposition = response.headers.get('content-disposition')
        if content_disposition:
            filename = unquote(content_disposition.split('filename=')[-1].strip('"'))
            save_path = f"{save_directory}/{filename}"
        else:
            save_path = f"{save_directory}/default_filename.ext"
            click.echo("Filename not provided in the response headers, saving as default_filename.ext")
        
        with open(save_path, 'wb') as f:
            for chunk in response.iter_content(chunk_size=1024):
                progress_bar.update(len(chunk))
                if chunk:
                    f.write(chunk)
        progress_bar.close()
        click.echo(f"File downloaded successfully and saved to {save_path}")
    elif response.status_code == 404:
        click.echo(f"Error: {response.json()['error']}")
    else:
        click.echo(f"Failed to download the file. Status code: {response.status_code}")

@cli.command()
@click.argument('file_path')
@click.argument('source')
def dataset_upload(file_path, source):
    try:
        with open(file_path, 'rb') as f:
            files = {'file': f}
            click.echo(f"Uploading {file_path}. Please wait and do not kill the process. This may take a while.")
            response = requests.post(f"{base_url}/datasets/upload", files=files)
            if response.status_code == 200:
                response_data = response.json()
                click.echo(f"Message: {response_data['message']}. Dataset name: {response_data['filename']}")
                response = requests.put(
                    f"{base_url}/datasets/{response_data['inserted_datasets_id']}", 
                    json={"Source": source}
                )
                if response.status_code == 200:
                    click.echo("Dataset updated successfully")
            elif response.status_code == 400:
                click.echo(f"Error: {response.json()['error']}")
            else:
                click.echo(f"Failed to upload the file. Status code: {response.status_code}")
    except FileNotFoundError:
        click.echo(f"File not found: {file_path}")
    except PermissionError:
        click.echo(f"Permission denied: {file_path}")
    except Exception as e:
        click.echo(f"An error occurred: {e}")

@cli.command()
@click.argument('file_path')
@click.argument('source')
def dataset_uploadBag(file_path, source):
    assert file_path.endswith('.bag'), "File must be a ROS bag file."
    try:
        with open(file_path, 'rb') as f:
            files = {'file': f}
            click.echo(f"Uploading {file_path}. Please wait and do not kill the process. This may take a while.")
            response = requests.post(f"{base_url}/datasets/uploadBag", files=files)
            if response.status_code == 200:
                response_data = response.json()
                click.echo(f"Message: {response_data['message']}.")

                for inserted in response_data['inserted_objects']:
                    response = requests.put(
                        f"{base_url}/datasets/{inserted[1]}", 
                        json={"Source": source}
                    )
                    if response.status_code == 200:
                        click.echo("Dataset updated successfully")
                        
            elif response.status_code == 400:
                click.echo(f"Error: {response.json()['error']}")
            else:
                click.echo(f"Failed to upload the file. Status code: {response.status_code}")
    except FileNotFoundError:
        click.echo(f"File not found: {file_path}")
    except PermissionError:
        click.echo(f"Permission denied: {file_path}")
    except Exception as e:
        click.echo(f"An error occurred: {e}")

@cli.command()
@click.argument('topics_file', type=click.Path(exists=True))
@click.argument('rosbag_file_name')
@click.option('--delete_rosbag', is_flag=True, help='Keep the rosbag file after recording')
def record_rosbag(topics_file, rosbag_file_name, delete_rosbag):

    with open(topics_file) as f:
        topics = [line.strip() for line in f.readlines()]

    if not topics:
        print(f"No topics found in {topics_file}")
        exit(1)

    topics = " ".join(topics)

    command = f"rosbag record {topics} -O {rosbag_file_name}"
    # command = "ping 127.0.0.1 -t"

    process = subprocess.Popen(command, shell=True)

    def stop_recording(sig, frame):
        print("Stopping rosbag recording")
        process.terminate()
        process.wait()
        print("Recording stopped")
        # send rosbag file through api
        # Add to here
        if delete_rosbag:
            print("Removing rosbag file")
            subprocess.run(f"rm {rosbag_file_name}", shell=True)
        exit(0)

    signal.signal(signal.SIGINT, stop_recording)

    print("Recording rosbag. Press Ctrl+C to stop recording")

    process.wait()