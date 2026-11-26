from mrider_navigation_sim.synchronization import ImageScanBuffer


def test_image_before_matching_scan_waits_without_invalid_status():
    b=ImageScanBuffer();b.scan(9.7,'old');b.image(10.,'image',1.)
    assert b.ready(1.05)==[]
    b.scan(10.,'matching')
    assert b.ready(1.06)==[('image','matching')]


def test_nearest_stamp_selected_even_if_newer_scan_arrived():
    b=ImageScanBuffer();b.scan(10.,'matching');b.scan(10.2,'newer')
    b.image(10.,'image',1.)
    assert b.ready(1.)==[('image','matching')]


def test_missing_scan_is_reported_after_bounded_wait():
    b=ImageScanBuffer();b.image(10.,'image',1.)
    assert not b.ready(1.1)
    assert b.ready(1.13)==[('image',None)]


def test_delay_does_not_increase_timestamp_tolerance():
    b=ImageScanBuffer();b.scan(9.8,'stale');b.image(10.,'image',1.)
    assert not b.ready(1.05)
    assert b.ready(1.13)==[('image','stale')]


def test_queue_is_bounded_and_keeps_image_order():
    b=ImageScanBuffer()
    for i in range(6):b.image(i,'image'+str(i),1.)
    assert b.ready(1.)==[('image0',None)] and len(b.images)==5
    assert [m for m,s in b.ready(1.2)]==['image'+str(i) for i in range(1,6)]
