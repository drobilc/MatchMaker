# Love Maker 2000
Love maker 2000 is a precious space for development of the extremly intelligent and polite robot named Erazem.

## TODO

### General

* brisanje neuporabljane kode
* izboljšaj komentiranje obstoječe kode
* premakni pogosto uporabljane funkcije v utils in jih zelo dobro komentiraj
* podaljšaj lifespan markerjev v utils
* detektorji naj počakajo, da je robot lokaliziran (preferrably ne z rospy.sleep)
* uporabi sinhronizacijo namesto localizerja
* costmap namesto static mapa (slike)
* ko je sinhronizacija implementirana, izbriši localizer
* ko pride nov svet, pazi na rotacijo mape

### Rings

* postavi approaching point za ring v njegovo bližino
* za approaching uporabi fine movement mode v `movement_controllerju`

### Cylinders

* sinhroniziraj point cloud
* fine tune filtriranje point clouda, da bo večkrat zaznal cilinder

### Movement controller

* izboljšaj računanje razdalj z upoštevanjem ovir (greš po vektorju od trenutne točke do cilja in če naletiš na zid, dodaš penalty k razdalji)
* definiraj vrstni red pobiranja approaching pointov
* briši cilje, ki jih dosežeš spotoma na poti na drug cilj, če niso approaching point in so orientirani podobno kot robot
* implementiraj fine movement mode (podobno kot pri lokalizaciji, pošiljaj twist message dokler ne prideš do zidu)

---

## Python libraries

For map_maker to run in python2, we need specific versions of the libraries. To install them, go to folder with `requirements.txt` and run command:

```cmd
pip install -r requirements.txt
```

---

Love maker will presumably be made following the scheme below.
![scheme](https://github.com/drobilc/rins_exercises/blob/master/love_maker_2000/new_scheme.png "scheme")
