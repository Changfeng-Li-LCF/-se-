import unittest
from unittest.mock import patch
import racecar_network as net


class NetworkTests(unittest.TestCase):
    def test_dynamic_subnet_and_scope(self):
        neighbors, addresses = net.subnet_candidates({'interfaces':[
            {'IPAddress':'192.168.123.14','PrefixLength':24},
            {'IPAddress':'169.254.1.2','PrefixLength':16}],
            'neighbors':['192.168.123.237','192.168.99.2','224.0.0.251']})
        self.assertEqual(neighbors,['192.168.123.237'])
        self.assertEqual(len(addresses),253)
        self.assertNotIn('192.168.123.14',addresses)
        self.assertIn('192.168.123.237',addresses)

    def test_large_subnet_bounded(self):
        _,addresses=net.subnet_candidates({'interfaces':[
            {'IPAddress':'10.20.30.14','PrefixLength':8}],'neighbors':[]})
        self.assertEqual(len(addresses),253)
        self.assertTrue(all(x.startswith('10.20.30.') for x in addresses))

    def test_open_ssh_is_not_identity(self):
        with patch.object(net,'reachable',side_effect=lambda h,p:h), \
             patch.object(net,'verified',side_effect=lambda h,c:h=='192.168.123.237'):
            self.assertEqual(net.select_verified(['192.168.123.1','192.168.123.237'],{}),'192.168.123.237')
        with patch.object(net,'reachable',return_value='192.168.123.1'), \
             patch.object(net,'verified',return_value=False):
            self.assertIsNone(net.select_verified(['192.168.123.1'],{}))

    def test_new_address_persisted_only_after_verification(self):
        config={'host':'192.168.1.2','auto_discover':True}
        with patch.object(net,'read_config',return_value=config), \
             patch.object(net,'lan_snapshot',return_value={'interfaces':[],'neighbors':[]}), \
             patch.object(net,'select_verified',side_effect=[None,'192.168.2.3']), \
             patch.object(net,'remember',return_value={'host':'192.168.2.3'}) as save:
            self.assertEqual(net.resolve()['host'],'192.168.2.3')
            save.assert_called_once_with('192.168.2.3',config)


if __name__=='__main__':unittest.main()
