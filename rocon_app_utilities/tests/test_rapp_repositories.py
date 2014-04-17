#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

from nose.tools import assert_equal, assert_false, assert_true
import os
import shutil
import tempfile

import rocon_console.console as console

from rocon_app_utilities.rapp_repositories import build_index, get_index, get_index_dest_prefix_for_base_paths, has_index, load_index, load_uris, save_uris

##############################################################################
# Tests
##############################################################################

pwd = os.getcwd()


def test_rapp_repositories():
    tempdir = tempfile.mkdtemp(suffix='', prefix='test_rapp_repositories_')

    # override default location of respoitory list file and cached index archives
    import rocon_app_utilities.rapp_repositories
    rocon_app_utilities.rapp_repositories._rapp_repositories_list_file = os.path.join(tempdir, 'rapp.list')

    uris = load_uris()
    assert_equal(uris, ['ROS_PACKAGE_PATH'])
    try:
        save_uris([])
        uris = load_uris()
        #assert_equal(uris, [])

        repo_path = os.path.join(os.path.dirname(__file__), 'test_rapp_repos')
        index = build_index([repo_path])
        assert_equal(index.raw_data.keys(), ['test_package_for_rapps/foo'])

        cache_exists = has_index(repo_path)
        assert_false(cache_exists)

        archive_prefix = get_index_dest_prefix_for_base_paths(repo_path)
        index.write_tarball(archive_prefix)

        cache_exists = has_index(repo_path)
        assert_true(cache_exists)

        archive_path = '%s.index.tar.gz' % archive_prefix
        index2 = load_index('file://%s' % archive_path)

        index2 = get_index(repo_path)
        assert_equal(index2.raw_data.keys(), ['test_package_for_rapps/foo'])

    finally:
        shutil.rmtree(tempdir)
