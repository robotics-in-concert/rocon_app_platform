#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
#################################################################################

try:
    from cStringIO import StringIO
except ImportError:
    from io import BytesIO as StringIO
import hashlib
import logging
import os
from rosdistro.loader import load_url
import rospkg
import rospkg.environment
import sys

from .indexer import RappIndexer, read_tarball

_rapp_repositories_list_file = os.path.join(rospkg.get_ros_home(), 'rocon/rapp.list')

logger = logging.getLogger('rapp_repositories')
logger.addHandler(logging.StreamHandler(sys.stderr))
#logger.setLevel(logging.DEBUG)


def load_uris():
    try:
        with open(_rapp_repositories_list_file, 'r') as h:
            uris = h.read().splitlines()
            logger.debug('load_uris() %s' % uris)
            return uris
    except IOError:
        return [rospkg.environment.ROS_PACKAGE_PATH]


def sanitize_uri(uri):
    if os.path.isfile(uri) or os.path.isdir(uri):
        uri = os.path.abspath(uri)
    return uri


def save_uris(uris):
    base_path = os.path.dirname(_rapp_repositories_list_file)
    if not os.path.exists(base_path):
        os.makedirs(base_path)
    if not os.path.exists(_rapp_repositories_list_file) and rospkg.environment.ROS_PACKAGE_PATH not in uris:
        uris.insert(0, rospkg.environment.ROS_PACKAGE_PATH)
    with open(_rapp_repositories_list_file, 'w') as h:
        logger.debug("save_uris(%s) to '%s'" % (uris, _rapp_repositories_list_file))
        for uri in uris:
            h.write('%s\n' % uri)


def uri2url(uri):
    if uri == rospkg.environment.ROS_PACKAGE_PATH:
        return get_ros_package_paths()
    if os.path.isabs(uri):
        if is_index(uri):
            return 'file://%s' % uri
        if os.path.isdir(uri):
            return [uri]
    return uri


def is_index(url_or_uri):
    return url_or_uri.endswith('.index.tar.gz')


def build_index(base_paths):
    logger.debug('build_index(%s)' % base_paths)
    assert isinstance(base_paths, list)
    combined_index = RappIndexer(raw_data={})
    for base_path in reversed(base_paths):
        index = RappIndexer(packages_path=base_path)
        combined_index.merge(index)
    combined_index.source = ':'.join(base_paths)
    return combined_index


def get_index_dest_prefix_for_base_paths(base_paths):
    base_path = _get_rapps_index_base_path()
    filename_prefix = _get_rapps_index_filename_prefix(base_paths)
    return os.path.join(base_path, filename_prefix)


def get_index(uri):
    logger.debug('get_index(%s)' % uri)
    if is_index(uri):
        url = uri2url(uri)
        return load_index(url)
    url = uri2url(uri)
    index_path = has_index(url)
    if index_path:
        index_url = 'file://%s' % index_path
        return load_index(index_url)
    return build_index(url)


def has_index(base_paths):
    dest_prefix = get_index_dest_prefix_for_base_paths(base_paths)
    path = '%s.index.tar.gz' % dest_prefix
    if os.path.exists(path):
        logger.debug('has_index(%s) %s' % (base_paths, path))
        return path
    path = '%s.index.yaml' % dest_prefix
    if os.path.exists(path):
        logger.debug('has_index(%s) %s' % (base_paths, path))
        return path
    logger.debug('has_index(%s) None' % base_paths)
    return None


def get_combined_index():
    logger.debug('get_combined_index()')
    combined_index = RappIndexer(raw_data={})
    uris = load_uris()
    for uri in reversed(uris):
        index = get_index(uri)
        combined_index.merge(index)
    return combined_index


def load_index(index_url):
    logger.debug('load_index(%s)' % index_url)
    if not index_url.endswith('.index.tar.gz'):
        raise NotImplementedError("The url of the index must end with '.index.tar.gz'")
    logger.debug('load_index() load gzipped tar index')
    tar_gz_str = load_url(index_url, skip_decode=True)
    tar_gz_stream = StringIO(tar_gz_str)
    index = read_tarball(fileobj=tar_gz_stream)
    index.source = index_url
    return index


def _get_rapps_index_base_path():
    return os.path.dirname(_rapp_repositories_list_file)


def _get_rapps_index_filename_prefix(source):
    digest = hashlib.md5(':'.join(source)).hexdigest()
    logger.debug("_get_rapps_index_filename_prefix(%s) hash '%s'" % (source, digest))
    return digest


def get_ros_package_paths():
    return rospkg.environment._compute_package_paths(None, rospkg.get_ros_package_path())
