import yaml
import rospkg


def parse_filename(filename):
    """Parse the filename, i.e. replace $(find ..) with path to package."""
    if ('$(find' in filename) and (')' in filename):
        filename = filename.replace('$(find ', '')  # assume filename starts with '$(find '
        idx_closing_bracket = filename.find(')')
        package = filename[:idx_closing_bracket]
        root = rospkg.RosPack().get_path(package)
        filename = filename.replace(f'{package})', root)
    return filename


def load_config(filename):
    """Loads a yaml config file. You can use $(find package-name)."""
    with open(parse_filename(filename), 'r') as configfile:
        return yaml.load(configfile, Loader=yaml.FullLoader)
