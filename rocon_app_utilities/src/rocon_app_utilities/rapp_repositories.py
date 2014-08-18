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

_rapp_repositories_list_file = os.path.join(rospkg.get_ros_home(), 'rocon', 'rapp', 'rapp.list')

logger = logging.getLogger('rapp_repositories')
logger.addHandler(logging.StreamHandler(sys.stderr))
#logger.setLevel(logging.DEBUG)


def load_uris():
    '''
      Loads the registered repository URIs from a configuration file.

      :returns: the list of URIs
      :rtype: [str]
    '''
    try:
        with open(_rapp_repositories_list_file, 'r') as h:
            uris = h.read().splitlines()
            logger.debug('load_uris() %s' % uris)
            return uris
    except IOError:
        return [rospkg.environment.ROS_PACKAGE_PATH]


def sanitize_uri(uri):
    '''
      Converts relative dir/file path into absolute.

      :param uri: the URI
      :type uri: str

      :returns: the sanitized URI
      :rtype: str
    '''
    if os.path.isfile(uri) or os.path.isdir(uri):
        uri = os.path.abspath(uri)
    return uri


def save_uris(uris):
    '''
      Save the repository URIs to a configuration.

      :param uris: the list of URIs
      :type uris: [str]
    '''
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
    '''
      Converts a URI into a URL.

      ROS_PACKAGE_PATH is being expanded into the actual list of directories.
      Local paths to index archives are converted into a file:// url.
      Single local directories are converted into a single element list.

      :param uri: the URI
      :type uri: str

      :returns: the sanitized URI
      :rtype: str
    '''
    if uri == rospkg.environment.ROS_PACKAGE_PATH:
        return get_ros_package_paths()
    if os.path.isabs(uri):
        if is_index(uri):
            return 'file://%s' % uri
        if os.path.isdir(uri):
            return [uri]
    return uri


def is_index(url_or_uri):
    '''
      Check if the URI or URL points to an index archive.

      :param url_or_uri: the URI or URL
      :type url_or_uri: str

      :returns: true, if URI or URL ends with '.index.tar.gz'
      :rtype: bool
    '''
    return url_or_uri.endswith('.index.tar.gz')


def build_index(base_paths, package_whitelist=None, package_blacklist=[]):
    '''
      Builds the index of rapps found under a list of base paths.

      :param base_paths: the list of base paths to crawl
      :type base_paths: [str]
      :param package_whitelist: list of target package list
      :type package_whitelist: [str]
      :param package_blacklist: list of blacklisted package
      :type package_blacklist: [str]

      :returns: the index
      :rtype: rocon_app_utilities.RappIndexer
    '''
    logger.debug('build_index(%s)' % base_paths)
    assert isinstance(base_paths, list)
    combined_index = RappIndexer(raw_data={})
    for base_path in reversed(base_paths):
        index = RappIndexer(packages_path=base_path, package_whitelist=package_whitelist, package_blacklist=package_blacklist)
        combined_index.merge(index)
    combined_index.source = ':'.join(base_paths)
    return combined_index


def get_index_dest_prefix_for_base_paths(base_paths):
    '''
      Returns the path of the cached index archive (without the suffix '.index.tar.gz).

      :param base_paths: the list of base paths
      :type base_paths: [str]

      :returns: the path prefix without the archive extension
      :rtype: str
    '''
    base_path = _get_rapps_index_base_path()
    filename_prefix = _get_rapps_index_filename_prefix(base_paths)
    return os.path.join(base_path, filename_prefix)


def get_index(uri, package_whitelist=None, package_blacklist=[]):
    '''
      Gets the index of the rapp repository identified by the URI.
      If the URI is a local folder it checks for the existance of a cached archive first.

      :param uri: the URI
      :type uri: str
      :param package_whitelist: list of target package list
      :type package_whitelist: [str]
      :param package_blacklist: list of blacklisted package
      :type package_blacklist: [str]

      :returns: the index
      :rtype: rocon_app_utilities.RappIndexer
    '''
    logger.debug('get_index(%s)' % uri)
    if is_index(uri):
        url = uri2url(uri)
        return load_index(url, package_whitelist=package_whitelist, package_blacklist=package_blacklist)
    url = uri2url(uri)
    index_path = has_index(url)
    if index_path:
        index_url = 'file://%s' % index_path
        return load_index(index_url, package_whitelist=package_whitelist, package_blacklist=package_blacklist)
    return build_index(url, package_whitelist=package_whitelist, package_blacklist=package_blacklist)


def has_index(base_paths):
    '''
      Returns the path of an existing cached index archive.

      :param base_paths: the list of base paths
      :type base_paths: [str]

      :returns: the path or None if no cached index archive exists
      :rtype: str
    '''
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


def get_combined_index(package_whitelist=None, package_blacklist=[]):
    '''
      Gets the combined index of the all registered rapp repositories.

      :param package_whitelist: list of target package list
      :type package_whitelist: [str]
      :param package_blacklist: list of blacklisted package
      :type package_blacklist: [str]

      :returns: the index
      :rtype: rocon_app_utilities.RappIndexer
    '''
    logger.debug('get_combined_index()')
    combined_index = RappIndexer(raw_data={})
    uris = load_uris()
    for uri in reversed(uris):
        index = get_index(uri, package_whitelist=package_whitelist, package_blacklist=package_blacklist)
        combined_index.merge(index)
    return combined_index


def load_index(index_url, package_whitelist=None, package_blacklist=[]):
    '''
      Loads the index for a URL pointing to an index archive.

      :param index_url: the URL
      :type index_url: str
      :param package_whitelist: list of target package list
      :type package_whitelist: [str]
      :param package_blacklist: list of blacklisted package
      :type package_blacklist: [str]

      :returns: the index
      :rtype: rocon_app_utilities.RappIndexer
    '''
    logger.debug('load_index(%s)' % index_url)
    if not index_url.endswith('.index.tar.gz'):
        raise NotImplementedError("The url of the index must end with '.index.tar.gz'")
    logger.debug('load_index() load gzipped tar index')
    tar_gz_str = load_url(index_url, skip_decode=True)
    tar_gz_stream = StringIO(tar_gz_str)
    index = read_tarball(fileobj=tar_gz_stream, package_whitelist=package_whitelist, package_blacklist=package_blacklist)
    index.source = index_url
    return index


def _get_rapps_index_base_path():
    '''
      Gets the folder in which the registered rapp repositories URIs as well as the cached index archives are stored.

      :returns: the path
      :rtype: str
    '''
    return os.path.dirname(_rapp_repositories_list_file)


def _get_rapps_index_filename_prefix(source):
    '''
      Gets filename prefix for a cached index archive (without the suffix '.index.tar.gz).

      :param source: the list of paths
      :type source: [str]

      :returns: the path
      :rtype: str
    '''
    digest = hashlib.md5(':'.join(source)).hexdigest()
    logger.debug("_get_rapps_index_filename_prefix(%s) hash '%s'" % (source, digest))
    return digest


def get_ros_package_paths():
    '''
      Get the list of paths from the ROS_PACKAGE_PATH environment variable.

      :returns: the list of paths
      :rtype: [str]
    '''
    return rospkg.environment._compute_package_paths(None, rospkg.get_ros_package_path())
